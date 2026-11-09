// ============================================================================
//  BajaLink — emulador de telemetria do Baja
// ----------------------------------------------------------------------------
//  Usado somente pelo sketch TX, quando BL_SOURCE == BL_SOURCE_EMU.
//
//  Gera exatamente o mesmo record JSON que o receptor STM32 entrega hoje ao
//  dashboard, nas MESMAS unidades brutas: roda em contagens de pulso (o
//  dashboard aplica mult 0,0625 e 0,076923), aceleração em contagens de
//  ~14 por g, bateria em mV, temperaturas em °C, combustível em contagens de
//  ADC. Isso é deliberado: o dashboard, os arquivos de calibração e os
//  limites de alerta já existentes continuam valendo sem ajuste, e um log
//  gravado com o emulador é indistinguível em formato de um log de pista.
//
//  O modelo não é um simulador de dinâmica veicular, e não pretende ser. Ele
//  é um gerador de carga com a estatística certa: um circuito de ~68 s
//  dividido em setores (retas, frenagens, curvas, um trecho de terreno
//  irregular), motor com atraso de primeira ordem no acelerador, velocidade
//  integrada com arrasto e frenagem, aceleração lateral tirada do raio de
//  curva, temperaturas com constante de tempo lenta puxadas pela carga
//  média, e tensão de bateria afundando com o consumo. O que importa para
//  validar o link é que a taxa, o tamanho do record e a faixa de cada
//  variável sejam realistas — e que o diagrama g-g do dashboard se preencha
//  como o de um carro de verdade, não como ruído branco.
// ============================================================================
#ifndef BL_EMU_H
#define BL_EMU_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#define BL_EMU_LAP_S        68.0f
#define BL_EMU_RPM_IDLE     1500.0f
#define BL_EMU_RPM_MAX      3700.0f
#define BL_EMU_V_MAX        13.0f    // m/s (~47 km/h)
#define BL_EMU_WHEELBASE    1.55f    // m, usado para o raio de curva
#define BL_EMU_G_COUNTS     14.0f    // contagens por g do acelerômetro
#define BL_EMU_T_AMB        28.0f

typedef struct {
  float t;            // s desde o boot
  float rpm;
  float v;            // m/s
  float v_prev;
  float load_avg;     // média móvel do acelerador (puxa as temperaturas)
  float t_motor;
  float t_cvt;
  float fuel;         // contagens de ADC
  float gy_prev;
  uint8_t estop;
  uint32_t rnd;
} bl_emu_t;

static bl_emu_t bl_emu;

// Ruído uniforme em [-1, 1]. LCG próprio para não depender de random() e
// manter o custo constante.
static inline float bl_emu_noise(void) {
  bl_emu.rnd = bl_emu.rnd * 1664525u + 1013904223u;
  return ((float)((bl_emu.rnd >> 8) & 0xFFFF) / 32768.0f) - 1.0f;
}

static inline void bl_emu_init(void) {
  memset(&bl_emu, 0, sizeof(bl_emu));
  bl_emu.rpm     = BL_EMU_RPM_IDLE;
  bl_emu.t_motor = BL_EMU_T_AMB;
  bl_emu.t_cvt   = BL_EMU_T_AMB - 6.0f;
  bl_emu.fuel    = 3400.0f;
  bl_emu.rnd     = 0xBA1A5AE0u;
}

static inline void bl_emu_set_estop(uint8_t on) { bl_emu.estop = on ? 1 : 0; }
static inline uint8_t bl_emu_get_estop(void)    { return bl_emu.estop; }

// ---------------------------------------------------------------------------
// Setores do circuito: acelerador, freio, esterço em graus, amplitude de
// irregularidade do piso.
// ---------------------------------------------------------------------------
static inline void bl_emu_sector(float p, float *tps, float *brk,
                                 float *steer, float *bump) {
  if      (p < 0.15f) { *tps = 1.00f; *brk = 0.00f; *steer =   0.0f; *bump = 0.20f; }
  else if (p < 0.22f) { *tps = 0.00f; *brk = 0.80f; *steer =  10.0f; *bump = 0.30f; }
  else if (p < 0.32f) { *tps = 0.35f; *brk = 0.10f; *steer =  28.0f; *bump = 0.30f; }
  else if (p < 0.45f) { *tps = 0.90f; *brk = 0.00f; *steer =   8.0f; *bump = 0.40f; }
  else if (p < 0.55f) { *tps = 0.60f; *brk = 0.20f; *steer = -24.0f; *bump = 0.40f; }
  else if (p < 0.70f) { *tps = 0.50f; *brk = 0.10f; *steer =  -5.0f; *bump = 1.40f; }
  else if (p < 0.82f) { *tps = 1.00f; *brk = 0.00f; *steer =   0.0f; *bump = 0.30f; }
  else if (p < 0.90f) { *tps = 0.45f; *brk = 0.30f; *steer = -20.0f; *bump = 0.50f; }
  else                { *tps = 0.50f; *brk = 0.25f; *steer =  22.0f; *bump = 0.50f; }
}

// ---------------------------------------------------------------------------
// Avança o modelo em dt segundos.
// ---------------------------------------------------------------------------
static inline void bl_emu_step(float dt) {
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
  bl_emu.t += dt;

  float p = fmodf(bl_emu.t, BL_EMU_LAP_S) / BL_EMU_LAP_S;
  float tps, brk, steer, bump;
  bl_emu_sector(p, &tps, &brk, &steer, &bump);

  // Suaviza as transições de setor para não gerar degraus impossíveis.
  tps   += 0.06f * bl_emu_noise();
  steer += 2.5f  * bl_emu_noise();
  if (tps < 0.0f) tps = 0.0f;
  if (tps > 1.0f) tps = 1.0f;

  if (bl_emu.estop) { tps = 0.0f; brk = 1.0f; }

  // Motor: primeira ordem no acelerador, com freio-motor sob frenagem.
  float rpm_target = BL_EMU_RPM_IDLE + tps * (BL_EMU_RPM_MAX - BL_EMU_RPM_IDLE);
  rpm_target -= brk * 420.0f;
  if (rpm_target < BL_EMU_RPM_IDLE * 0.75f) rpm_target = BL_EMU_RPM_IDLE * 0.75f;
  bl_emu.rpm += (rpm_target - bl_emu.rpm) * (dt / 0.35f);

  // Velocidade: a CVT faz a roda seguir o motor com escorregamento, então a
  // velocidade persegue o regime com constante de tempo maior que a do motor.
  float v_target = (bl_emu.rpm - BL_EMU_RPM_IDLE) /
                   (BL_EMU_RPM_MAX - BL_EMU_RPM_IDLE) * BL_EMU_V_MAX;
  if (v_target < 0.0f) v_target = 0.0f;
  bl_emu.v_prev = bl_emu.v;
  bl_emu.v += (v_target - bl_emu.v) * (dt / 1.20f);
  bl_emu.v -= brk * 6.0f * dt;              // frenagem como desaceleração direta
  bl_emu.v -= 0.02f * bl_emu.v * dt;        // arrasto
  if (bl_emu.v < 0.0f) bl_emu.v = 0.0f;

  // Carga média: é ela que aquece motor e CVT, não o acelerador instantâneo.
  bl_emu.load_avg += (tps - bl_emu.load_avg) * (dt / 12.0f);

  float tgt_motor = 70.0f + 35.0f * bl_emu.load_avg;
  float tgt_cvt   = 55.0f + 45.0f * bl_emu.load_avg;
  bl_emu.t_motor += (tgt_motor - bl_emu.t_motor) * (dt / 25.0f);
  bl_emu.t_cvt   += (tgt_cvt   - bl_emu.t_cvt)   * (dt / 18.0f);

  bl_emu.fuel -= 0.35f * dt * (0.3f + tps);
  if (bl_emu.fuel < 1200.0f) bl_emu.fuel = 3400.0f;   // reabastece e recomeça

  bl_emu.gy_prev = (bl_emu.v - bl_emu.v_prev) / dt;
}

// ---------------------------------------------------------------------------
// Serializa o estado atual como record JSON, no formato que o dashboard já
// consome. Devolve o número de bytes escritos (sem o '\n').
// ---------------------------------------------------------------------------
static inline int bl_emu_format(char *out, size_t cap,
                                uint32_t ts_ms, uint32_t seq) {
  float p = fmodf(bl_emu.t, BL_EMU_LAP_S) / BL_EMU_LAP_S;
  float tps, brk, steer, bump;
  bl_emu_sector(p, &tps, &brk, &steer, &bump);
  if (bl_emu.estop) { tps = 0.0f; brk = 1.0f; }

  float kmh = bl_emu.v * 3.6f;

  // Aceleração longitudinal a partir da variação de velocidade.
  float ax_g = bl_emu.gy_prev / 9.81f;

  // Lateral pelo raio de curva: R = L / tan(delta), limitado para não
  // explodir em reta.
  float st = fabsf(steer);
  float R = (st < 0.5f) ? 1.0e4f : (BL_EMU_WHEELBASE / tanf(st * 0.01745f));
  if (R < 4.0f) R = 4.0f;
  float ay_g = (bl_emu.v * bl_emu.v / R) / 9.81f;
  if (steer > 0.0f) ay_g = -ay_g;

  float az_g = 1.0f + bump * 0.45f * bl_emu_noise();

  // Velocidades angulares: guinada pelo raio, rolagem e arfagem pelo piso.
  float gz = (R > 1.0e3f) ? 0.0f : (bl_emu.v / R) * 57.2958f;
  if (steer < 0.0f) gz = -gz;
  float gx = bump * 22.0f * bl_emu_noise();
  float gy = bump * 14.0f * bl_emu_noise() - ax_g * 6.0f;

  float vbat = 12700.0f - 500.0f * tps - 180.0f * brk + 25.0f * bl_emu_noise();
  if (vbat < 11800.0f) vbat = 11800.0f;

  int roda_fr = (int)lroundf(kmh / 0.0625f);     // contagens -> mult 0,0625
  int roda_rr = (int)lroundf(kmh / 0.076923f);   // contagens -> mult 0,076923

  return snprintf(out, cap,
    "{\"ts\":%lu,\"seq\":%lu,"
    "\"roda_fr\":%d,\"roda_rr\":%d,\"motor_rpm\":%d,"
    "\"freio_d\":%d,\"freio_t\":%d,"
    "\"ax\":%d,\"ay\":%d,\"az\":%d,"
    "\"gx\":%d,\"gy\":%d,\"gz\":%d,"
    "\"bateria_mv\":%d,\"motor_temp\":%d,\"cvt_temp\":%d,"
    "\"combustivel\":%d,\"estop\":%d}",
    (unsigned long)ts_ms, (unsigned long)seq,
    roda_fr, roda_rr, (int)lroundf(bl_emu.rpm),
    (int)lroundf(brk * 42.0f), (int)lroundf(brk * 30.0f),
    (int)lroundf(ax_g * BL_EMU_G_COUNTS),
    (int)lroundf(ay_g * BL_EMU_G_COUNTS),
    (int)lroundf(az_g * BL_EMU_G_COUNTS),
    (int)lroundf(gx), (int)lroundf(gy), (int)lroundf(gz),
    (int)lroundf(vbat), (int)lroundf(bl_emu.t_motor),
    (int)lroundf(bl_emu.t_cvt),
    (int)lroundf(bl_emu.fuel), bl_emu.estop);
}

#endif // BL_EMU_H
