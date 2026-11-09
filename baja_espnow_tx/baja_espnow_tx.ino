// ============================================================================
//  Telemetria Baja SAE — nó sensor (transmissor BajaLink / ESP-NOW)
//  Elias Enns, Wesley Machado — trilha formativa de Internet das Coisas
// ----------------------------------------------------------------------------
//  Papel deste nó
//
//  Substitui o caminho LoRa na captura de dados. A central STM32 do carro
//  passa a despejar os records de telemetria na USART3, que estava livre, e
//  este ESP32 os leva até o box por ESP-NOW. O ganho não está no alcance —
//  LoRa ganha disso com folga — e sim na banda: o caminho LoRa atual sustenta
//  algo em torno de 2 records/s, o que basta para acompanhar temperatura e
//  tensão, mas joga fora exatamente o que interessa numa análise de dinâmica
//  e de estrutura. A 50 records/s já se vê o transitório de uma frenagem e o
//  conteúdo de frequência de um impacto de suspensão.
//
//  Modo de operação (ver bloco BL_SOURCE abaixo)
//
//    BL_SOURCE_EMU   — padrão. O record é gerado aqui por um modelo de volta,
//                      no mesmo formato e nas mesmas unidades brutas do
//                      receptor STM32. Permite exercitar e medir o link sem
//                      o carro e sem a placa.
//    BL_SOURCE_UART  — o record vem da USART3 do STM32 pela UART1 deste
//                      ESP32, uma linha JSON por record.
//
//  O que o link acrescenta ao ESP-NOW
//
//  ESP-NOW entrega quadro íntegro ou não entrega nada, e não ordena, não
//  retransmite a nível de aplicação, nem guarda o que não passou. Para um
//  datalogger isso é insuficiente. Esta camada acrescenta número de
//  sequência, ACK cumulativo com extensão seletiva, retransmissão rápida por
//  lacuna confirmada e por RTO estimado, janela de congestionamento
//  adaptativa e um buffer store-and-forward em RAM que transforma queda de
//  link em latência em vez de lacuna. A especificação do formato de fio está
//  em docs/BAJALINK_PROTOCOLO.md.
//
//  Placa: ESP32-S3-DevKitC-1. Ferramentas > "USB CDC On Boot" conforme a
//  porta usada para o console (ver README.md).
// ============================================================================

#include "bl_config.h"
#include "bl_proto.h"
#include "bl_radio.h"
#include "bl_store.h"
#include "bl_emu.h"

// ============================================================================
//  FONTE DE DADOS — escolha aqui
// ============================================================================
#define BL_SOURCE_EMU   0
#define BL_SOURCE_UART  1

#ifndef BL_SOURCE
  #define BL_SOURCE  BL_SOURCE_EMU      // <<< trocar para BL_SOURCE_UART no hardware novo
#endif

#if BL_SOURCE == BL_SOURCE_UART
  #define BL_SRC_NAME  "uart"
#else
  #define BL_SRC_NAME  "emu"
#endif

// ============================================================================
//  Estado
// ============================================================================

// ---- Fila entre o callback do rádio e o laço principal --------------------
// Nada de trabalho pesado dentro do callback: ele roda no contexto da task de
// Wi-Fi, e bloquear ali derruba recepções. O callback só copia e enfileira.
typedef struct {
  uint8_t len;
  int8_t  rssi;
  uint8_t mac[6];
  uint8_t buf[BL_FRAME_MAX];
} bl_evt_t;

static QueueHandle_t      q_rx;
static SemaphoreHandle_t  sem_hw;    // créditos da fila de transmissão do rádio

// ---- Associação -----------------------------------------------------------
enum { LS_DISCOVER = 0, LS_UP = 1 };
static uint8_t  g_ls          = LS_DISCOVER;
static uint8_t  g_peer[6]     = {0};
static bool     g_have_peer   = false;
static uint32_t g_boot_id     = 0;
static uint32_t g_peer_boot   = 0;
static uint32_t g_last_rx_ms  = 0;
static uint32_t g_last_hello  = 0;
static uint32_t g_link_drops  = 0;

static const uint8_t g_ch_list[BL_CHANNEL_LIST_LEN] = BL_CHANNEL_LIST;
static uint8_t  g_ch_idx      = 0;
static uint32_t g_ch_since    = 0;

// ---- Controle de congestionamento e RTT -----------------------------------
static uint16_t g_cwnd        = BL_CWND_INIT;
static uint16_t g_acked_run   = 0;
static float    g_srtt        = 0.0f;
static float    g_rttvar      = 0.0f;
static uint32_t g_rto         = 120;
static uint16_t g_recover_seq = 0;      // fim da época de recuperação atual
static bool     g_in_recovery = false;

// ---- Contadores -----------------------------------------------------------
static uint32_t g_records     = 0;
static uint32_t g_frames      = 0;
static uint32_t g_retx        = 0;
static uint32_t g_mac_fail    = 0;
static uint32_t g_since_reqack= 0;
static int8_t   g_rssi_ack    = 0;
static uint32_t g_peer_rx     = 0;
static uint32_t g_peer_dups   = 0;
static uint32_t g_peer_gaps   = 0;
static uint32_t g_uart_overrun= 0;

// ---- Taxa da fonte --------------------------------------------------------
static uint16_t g_src_hz      = 50;
static uint32_t g_src_win_t0  = 0;
static uint32_t g_src_win_n   = 0;
static uint16_t g_src_hz_meas = 0;

// ---- Empacotamento --------------------------------------------------------
static uint8_t  g_stage[BL_MAX_PAYLOAD];
static uint16_t g_stage_len   = 0;
static uint8_t  g_stage_recs  = 0;
static uint32_t g_stage_t0    = 0;

// ---- Fonte ----------------------------------------------------------------
static uint32_t g_rec_seq     = 0;
static uint32_t g_emu_next_us = 0;
#if BL_SOURCE == BL_SOURCE_UART
static char     g_uline[BL_MAX_RECORD];
static uint16_t g_ulen        = 0;
#endif

// ============================================================================
//  Callbacks do rádio
// ============================================================================

BL_RECV_CB(on_espnow_recv) {
  if (len <= 0 || len > BL_FRAME_MAX) return;
  bl_evt_t e;
  e.len  = (uint8_t)len;
  e.rssi = BL_RECV_RSSI;
  memcpy(e.mac, BL_RECV_MAC, 6);
  memcpy(e.buf, data, (size_t)len);
  xQueueSend(q_rx, &e, 0);     // sem espera: melhor perder que travar o rádio
}

BL_SEND_CB(on_espnow_sent) {
  if (status != ESP_NOW_SEND_SUCCESS) g_mac_fail++;
  xSemaphoreGive(sem_hw);      // devolve o crédito de fila
}

// ============================================================================
//  Transmissão de um quadro
// ============================================================================

// O ESP-NOW tem fila interna curta e devolve ESP_ERR_ESPNOW_NO_MEM quando se
// enfileira rápido demais. O semáforo contador limita os envios em voo a
// BL_HW_QUEUE, o que evita o erro e, de passagem, dá o controle de ritmo de
// graça: o crédito só volta quando o rádio confirma o quadro anterior.
static bool bl_tx_frame(uint8_t type, uint8_t flags, uint16_t seq,
                        const void *pl, uint16_t len, const uint8_t *dst) {
  static uint8_t frame[BL_FRAME_MAX];
  size_t n = bl_build(frame, type, flags, seq, pl, len);
  if (n == 0) return false;
  if (xSemaphoreTake(sem_hw, 0) != pdTRUE) return false;
  if (esp_now_send(dst, frame, n) != ESP_OK) {
    xSemaphoreGive(sem_hw);
    return false;
  }
  return true;
}

// ============================================================================
//  Estimador de RTT e RTO (RFC 6298)
// ============================================================================

static void bl_rtt_update(uint32_t sample_ms) {
  if (sample_ms > BL_RTO_MAX_MS * 4) return;      // descarta aberrações
  if (g_srtt <= 0.0f) {
    g_srtt   = (float)sample_ms;
    g_rttvar = (float)sample_ms / 2.0f;
  } else {
    g_rttvar = 0.75f * g_rttvar + 0.25f * fabsf(g_srtt - (float)sample_ms);
    g_srtt   = 0.875f * g_srtt + 0.125f * (float)sample_ms;
  }
  // Margem do RTO.
  //
  // A forma pura do RFC 6298, srtt + 4*rttvar, tem um ponto cego: num link
  // muito estável rttvar tende a zero e o RTO converge para o próprio RTT
  // médio. A partir daí qualquer jitter de uma amostra estoura o
  // temporizador e produz retransmissão espúria — gasto de ar e de tentativas
  // sem nenhuma perda real. No teste de integração isso apareceu de forma
  // escandalosa: canal sem perda nenhuma e quase uma retransmissão por
  // quadro, porque o RTO havia convergido para exatamente o RTT medido e a
  // verificação de expiração rodava no mesmo milissegundo em que o ACK
  // chegava.
  //
  // A correção garante margem por dois caminhos ao mesmo tempo: um piso
  // multiplicativo sobre o srtt, que escala junto com o RTT, e uma folga
  // fixa, que cobre a granularidade do relógio e o atraso do agrupamento de
  // ACK no gateway.
  float rto = g_srtt + 4.0f * g_rttvar;
  float floor_mult = g_srtt * 1.25f;
  if (rto < floor_mult) rto = floor_mult;
  rto += (float)BL_RTO_SLACK_MS;
  if (rto < BL_RTO_MIN_MS) rto = BL_RTO_MIN_MS;
  if (rto > BL_RTO_MAX_MS) rto = BL_RTO_MAX_MS;
  g_rto = (uint32_t)rto;
}

// Uma redução de janela por EVENTO de perda, não por ACK.
//
// Esta foi a correção mais importante que o teste de integração produziu. A
// primeira versão reduzia a janela sempre que um ACK trazia bits seletivos
// acesos, protegida apenas por uma guarda de tempo. Está errado: bits acesos
// significam "chegou algo fora de ordem", condição que persiste durante toda
// a lacuna, e não um novo evento de perda. Com ACK a cada ~15 ms e guarda de
// 25 ms, a janela era cortada em quase todo ACK, desabava para o mínimo e não
// voltava a subir. A vazão ficava abaixo da taxa de aquisição e o buffer
// vivia estourado — com 15 % de perda no canal, a entrega caía para 85 % em
// vez dos 100 % que a retransmissão deveria garantir.
//
// A regra correta é a época de recuperação do NewReno: ao detectar perda,
// marca-se o ponto atual da janela de transmissão. Qualquer lacuna até esse
// ponto pertence ao mesmo evento e não reduz de novo. Sai-se da recuperação
// quando a base cumulativa passa dele.
static void bl_on_loss(void) {
  if (g_in_recovery) return;
  g_in_recovery = true;
  g_recover_seq = bl_sf.next_seq;

  // Contrapressão do buffer.
  //
  // Reduzir a janela com backlog acumulado é contraproducente neste link. A
  // concorrência real no ar já está limitada por BL_HW_QUEUE, que é o que
  // protege a fila do rádio; a janela do ARQ dimensiona o pipeline, não o
  // acesso ao meio. E a fonte de dados é de taxa fixa: ela não desacelera
  // porque o link piorou. Com o buffer acima da metade, fechar a janela
  // atrasa a drenagem e aproxima o estouro, que é uma perda definitiva —
  // trocar perda definitiva por cortesia com um meio que ninguém mais está
  // usando é um mau negócio.
  //
  // Fora dessa condição, o AIMD normal vale: a redução evita insistir num
  // meio degradado e desperdiçar ar com retransmissão.
  if ((uint32_t)bl_store_used() * 2 >= (uint32_t)bl_sf.n) {
    g_acked_run = 0;
    return;
  }

  g_cwnd = (uint16_t)(g_cwnd / 2);
  if (g_cwnd < BL_CWND_MIN) g_cwnd = BL_CWND_MIN;
  g_acked_run = 0;
}

// ============================================================================
//  Empacotamento de records
// ============================================================================

static void bl_stage_flush(void) {
  if (g_stage_len == 0) return;
  bl_store_push(g_stage, g_stage_len, g_stage_recs, 0, NULL);
  g_stage_len  = 0;
  g_stage_recs = 0;
}

// Records maiores que um payload vão fragmentados, um pedaço por quadro.
// Só o último fragmento conta como record, para que o descarte por estouro
// não contabilize o mesmo dado várias vezes.
static void bl_push_fragmented(const uint8_t *p, uint16_t len) {
  uint16_t off = 0;
  while (off < len) {
    uint16_t chunk = (uint16_t)(len - off);
    if (chunk > BL_MAX_PAYLOAD) chunk = BL_MAX_PAYLOAD;
    bool last = (uint16_t)(off + chunk) >= len;
    uint8_t f = BL_F_FRAG | (last ? 0 : BL_F_MORE);
    bl_store_push(p + off, chunk, last ? 1 : 0, f, NULL);
    off = (uint16_t)(off + chunk);
  }
}

static void bl_push_record(const uint8_t *rec, uint16_t len) {
  if (len == 0) return;
  g_records++;
  g_src_win_n++;

  if (len > BL_REC_MAX_INLINE) {
    bl_stage_flush();
    bl_push_fragmented(rec, len);
    return;
  }
  if ((uint16_t)(g_stage_len + 1 + len) > BL_MAX_PAYLOAD) bl_stage_flush();
  if (g_stage_len == 0) g_stage_t0 = millis();

  g_stage[g_stage_len++] = (uint8_t)len;
  memcpy(g_stage + g_stage_len, rec, len);
  g_stage_len = (uint16_t)(g_stage_len + len);
  g_stage_recs++;

  if (g_stage_recs >= BL_AGG_MAX_RECORDS) bl_stage_flush();
}

// ============================================================================
//  Fonte de dados
// ============================================================================

#if BL_SOURCE == BL_SOURCE_EMU
static void bl_poll_source(void) {
  uint32_t period_us = 1000000UL / (g_src_hz ? g_src_hz : 1);
  uint32_t now = micros();
  int guard = 0;
  while ((int32_t)(now - g_emu_next_us) >= 0 && guard++ < 8) {
    bl_emu_step((float)period_us / 1000000.0f);
    char rec[BL_MAX_RECORD];
    int n = bl_emu_format(rec, sizeof(rec), millis(), ++g_rec_seq);
    if (n > 0 && n < (int)sizeof(rec)) bl_push_record((const uint8_t *)rec, (uint16_t)n);
    g_emu_next_us += period_us;
    now = micros();
  }
  // Se o laço atrasou muito (drenagem agressiva de buffer, por exemplo),
  // realinha a base de tempo em vez de tentar recuperar as amostras.
  if ((int32_t)(now - g_emu_next_us) > (int32_t)(period_us * 8)) g_emu_next_us = now;
}
#else
static void bl_poll_source(void) {
  // Orçamento de bytes por passagem: ler a UART até esvaziar deixaria o ARQ
  // sem CPU justamente quando a fonte está rápida.
  int budget = 2048;
  while (Serial1.available() && budget-- > 0) {
    int c = Serial1.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') {
      if (g_ulen > 0) {
        bl_push_record((const uint8_t *)g_uline, g_ulen);
        g_rec_seq++;
        g_ulen = 0;
      }
    } else if (g_ulen < sizeof(g_uline)) {
      g_uline[g_ulen++] = (char)c;
    } else {
      g_uart_overrun++;   // linha sem terminador: descarta e ressincroniza
      g_ulen = 0;
    }
  }
}
#endif

// ============================================================================
//  Tratamento de ACK
// ============================================================================

static void bl_handle_ack(const bl_ack_t *a, int8_t rssi, uint32_t now) {
  g_rssi_ack   = rssi;
  g_last_rx_ms = now;

  // Amostra de RTT só de quadro transmitido uma única vez (algoritmo de
  // Karn): usar o tempo de uma retransmissão contamina o estimador, porque
  // não se sabe qual das cópias foi confirmada.
  bl_slot_t *echoed = bl_store_get(a->echo_seq);
  if (echoed && echoed->tries == 1 && echoed->t_last_ms)
    bl_rtt_update((uint32_t)(now - echoed->t_last_ms));

  uint16_t freed = bl_store_ack_upto(a->ack_base);

  // Fim da época de recuperação: a base passou do ponto onde a perda foi
  // detectada, logo a próxima lacuna é um evento novo.
  if (g_in_recovery && bl_seq_ge(bl_sf.base_seq, g_recover_seq))
    g_in_recovery = false;

  if (freed && !g_in_recovery) {
    g_acked_run = (uint16_t)(g_acked_run + freed);
    if (g_acked_run >= g_cwnd) {           // crescimento aditivo: +1 por RTT
      g_acked_run = 0;
      if (g_cwnd < BL_CWND_MAX) g_cwnd++;
    }
  }

  // Extensão seletiva: libera o que chegou fora de ordem e, como há bits
  // acima da base, trata a própria base como lacuna confirmada e retransmite
  // na hora, sem esperar o RTO.
  if (a->sack_bits) {
    for (uint8_t i = 0; i < 32; i++)
      if (a->sack_bits & (1u << i))
        bl_store_ack_one((uint16_t)(a->ack_base + 1 + i));

    // No máximo uma retransmissão rápida por RTT para a mesma lacuna. Sem
    // essa guarda, cada ACK recebido durante a lacuna gerava uma cópia: o
    // quadro esgotava as BL_MAX_TRIES tentativas em menos de 200 ms e era
    // abandonado, convertendo uma perda recuperável em lacuna definitiva.
    uint32_t min_gap = (g_srtt > 0.0f) ? (uint32_t)g_srtt : BL_RTO_MIN_MS;
    if (min_gap < BL_RTO_MIN_MS) min_gap = BL_RTO_MIN_MS;

    bl_slot_t *hole = bl_store_get(a->ack_base);
    if (hole && hole->tries > 0 && hole->tries < BL_MAX_TRIES && g_have_peer &&
        (uint32_t)(now - hole->t_last_ms) >= min_gap) {
      if (bl_tx_frame(BL_T_DATA, (uint8_t)(hole->flags | BL_F_RETX),
                      a->ack_base, hole->data, hole->len, g_peer)) {
        hole->tries++;
        hole->t_last_ms = now;
        g_retx++;
        bl_on_loss();
      }
    }
  }

  g_peer_rx   = a->rx_frames;
  g_peer_dups = a->rx_dups;
  g_peer_gaps = a->rx_gaps;
}

// ============================================================================
//  Recepção
// ============================================================================

static void bl_service_rx(uint32_t now) {
  bl_evt_t e;
  int guard = 0;
  while (xQueueReceive(q_rx, &e, 0) == pdTRUE && guard++ < 16) {
    bl_hdr_t h;
    const uint8_t *pl;
    if (!bl_parse(e.buf, e.len, &h, &pl)) continue;   // magic/versão/CRC

    switch (h.type) {
      case BL_T_HELLO_ACK: {
        if (h.len < sizeof(bl_hello_t)) break;
        bl_hello_t hi;
        memcpy(&hi, pl, sizeof(hi));
        if (hi.role != 1) break;                      // só o gateway responde

        bool is_new = !g_have_peer || memcmp(g_peer, e.mac, 6) != 0
                                   || hi.boot_id != g_peer_boot;
        memcpy(g_peer, e.mac, 6);
        g_peer_boot = hi.boot_id;
        if (is_new) {
          bl_peer_add(g_peer, BL_ENCRYPT ? true : false);
          Serial.printf("[link] gateway %s canal %u %s\n",
                        bl_mac_str(g_peer), bl_radio_get_channel(), bl_rate_name());
        }
        g_have_peer  = true;
        g_ls         = LS_UP;
        g_last_rx_ms = now;
        break;
      }
      case BL_T_ACK: {
        if (h.len < sizeof(bl_ack_t)) break;
        bl_ack_t a;
        memcpy(&a, pl, sizeof(a));
        bl_handle_ack(&a, e.rssi, now);
        break;
      }
      default: break;
    }
  }
}

// ============================================================================
//  Máquina de estados do link
// ============================================================================

static void bl_service_link(uint32_t now) {
  if (g_ls == LS_UP && (uint32_t)(now - g_last_rx_ms) > BL_LINK_LOST_MS) {
    g_ls = LS_DISCOVER;
    g_link_drops++;
    g_ch_since = now;
    g_cwnd = BL_CWND_INIT;     // janela não sobrevive a uma queda
    g_in_recovery = false;
    Serial.printf("[link] caiu (%lu quedas) — buffer segurando %u quadros\n",
                  (unsigned long)g_link_drops, bl_store_used());
  }

  if (g_ls != LS_DISCOVER) return;

  // Varredura de canal. Quem varre é o sensor; o gateway fica parado no canal
  // de casa. Se os dois varressem, poderiam nunca se cruzar.
  if ((uint32_t)(now - g_ch_since) > BL_CH_DWELL_MS) {
    g_ch_idx = (uint8_t)((g_ch_idx + 1) % BL_CHANNEL_LIST_LEN);
    bl_radio_set_channel(g_ch_list[g_ch_idx]);
    g_ch_since = now;
  }

  if ((uint32_t)(now - g_last_hello) >= BL_HELLO_PERIOD_MS) {
    g_last_hello = now;
    bl_hello_t hi;
    hi.boot_id   = g_boot_id;
    hi.proto_ver = BL_PROTO_VER;
    hi.role      = 0;
    hi.channel   = bl_radio_get_channel();
    hi.lr_mode   = BL_LONG_RANGE;
    bl_tx_frame(BL_T_HELLO, 0, 0, &hi, sizeof(hi), BL_BCAST);
  }
}

// ============================================================================
//  ARQ
// ============================================================================

static void bl_service_arq(uint32_t now) {
  // Limpa a cabeça da janela: slots já liberados por ACK seletivo e quadros
  // que esgotaram as tentativas. Sem isso, um único quadro irrecuperável
  // trava a drenagem de tudo que está atrás dele.
  for (;;) {
    if (bl_store_used() == 0) break;
    bl_slot_t *head = &bl_sf.slots[bl_sf.base_seq % bl_sf.n];
    if (!head->used) { bl_sf.base_seq++; continue; }
    if (head->tries >= BL_MAX_TRIES &&
        (uint32_t)(now - head->t_last_ms) >= g_rto) {
      bl_store_giveup_head();
      continue;
    }
    break;
  }

  if (g_ls != LS_UP || !g_have_peer) return;

  uint16_t used = bl_store_used();
  if (used == 0) return;
  if (used > bl_sf.n) used = bl_sf.n;

  // Retransmissões por RTO, com recuo exponencial por quadro.
  //
  // A definição de "em voo" importa muito aqui, e a primeira versão errou.
  // Contar como em voo todo slot já enviado e não confirmado parece natural,
  // mas o ACK é cumulativo e a extensão seletiva só alcança 32 sequências: com
  // uma lacuna parada na base, centenas de slots ficam eternamente "enviados e
  // não confirmados". A contagem estourava a janela, nenhum quadro novo saía,
  // e o buffer passava a descartar por congestionamento inventado. No teste de
  // integração isso aparecia como cwnd travada no piso e entrega de 85 % com
  // 15 % de perda no canal.
  //
  // Em voo é o que foi transmitido e ainda está dentro do seu RTO. Passado o
  // RTO, o quadro não está em voo: está aguardando retransmissão, e não deve
  // ocupar lugar na janela.
  uint16_t inflight = 0;
  for (uint16_t i = 0; i < used; i++) {
    uint16_t seq = (uint16_t)(bl_sf.base_seq + i);
    bl_slot_t *s = bl_store_get(seq);
    if (!s || s->tries == 0) continue;

    uint8_t shift = (uint8_t)((s->tries > 5) ? 4 : (s->tries - 1));
    uint32_t rto = g_rto << shift;
    if (rto > BL_RTO_MAX_MS) rto = BL_RTO_MAX_MS;

    if ((uint32_t)(now - s->t_last_ms) < rto) { inflight++; continue; }
    if (s->tries >= BL_MAX_TRIES) continue;

    if (bl_tx_frame(BL_T_DATA, (uint8_t)(s->flags | BL_F_RETX),
                    seq, s->data, s->len, g_peer)) {
      s->tries++;
      s->t_last_ms = now;
      g_retx++;
      inflight++;
      bl_on_loss();
    } else {
      break;    // sem crédito de rádio: tenta na próxima passagem
    }
  }

  // Quadros novos, até o limite da janela de congestionamento.
  for (uint16_t i = 0; i < used && inflight < g_cwnd; i++) {
    uint16_t seq = (uint16_t)(bl_sf.base_seq + i);
    bl_slot_t *s = bl_store_get(seq);
    if (!s || s->tries != 0) continue;

    uint8_t f = s->flags;
    if ((++g_since_reqack % BL_REQACK_EVERY) == 0) f |= BL_F_REQACK;

    if (!bl_tx_frame(BL_T_DATA, f, seq, s->data, s->len, g_peer)) break;
    s->tries      = 1;
    s->t_first_ms = now;
    s->t_last_ms  = now;
    g_frames++;
    inflight++;
  }
}

// ============================================================================
//  Telemetria do próprio link, espelhada no gateway
// ============================================================================

static void bl_service_stat(uint32_t now) {
  static uint32_t last = 0;
  if (g_ls != LS_UP || !g_have_peer) return;
  if ((uint32_t)(now - last) < BL_STAT_PERIOD_MS) return;
  last = now;

  bl_stat_t s;
  memset(&s, 0, sizeof(s));
  s.tx_records  = g_records;
  s.tx_frames   = g_frames;
  s.tx_retx     = g_retx;
  s.tx_dropped  = bl_sf.dropped;
  s.tx_giveup   = bl_sf.giveup;
  s.tx_base_seq = bl_sf.base_seq;
  s.buf_used    = bl_store_used();
  s.buf_size    = bl_sf.n;
  s.cwnd        = g_cwnd;
  s.srtt_ms     = (uint16_t)g_srtt;
  s.rto_ms      = (uint16_t)g_rto;
  s.src_hz      = g_src_hz_meas;
  s.rssi_ack    = g_rssi_ack;
  s.channel     = bl_radio_get_channel();
  s.lr_mode     = BL_LONG_RANGE;
  s.src         = (BL_SOURCE == BL_SOURCE_UART) ? 1 : 0;
  s.psram       = bl_sf.psram;
  s.uptime_s    = now / 1000;

  bl_tx_frame(BL_T_STAT, 0, 0, &s, sizeof(s), g_peer);
}

// ============================================================================
//  Console
// ============================================================================

static void bl_print_stats(void) {
  uint32_t sent = g_frames + g_retx;
  float retx_pct = sent ? (100.0f * (float)g_retx / (float)sent) : 0.0f;
  Serial.println(F("---- BajaLink TX ----"));
  Serial.printf("estado      : %s\n", g_ls == LS_UP ? "ASSOCIADO" : "procurando");
  Serial.printf("gateway     : %s\n", g_have_peer ? bl_mac_str(g_peer) : "-");
  Serial.printf("canal/taxa  : %u / %s\n", bl_radio_get_channel(), bl_rate_name());
  Serial.printf("fonte       : %s  %u Hz nominal  %u Hz medido\n",
                BL_SRC_NAME, g_src_hz, g_src_hz_meas);
  Serial.printf("records     : %lu\n", (unsigned long)g_records);
  Serial.printf("quadros     : %lu novos  %lu retx (%.2f%%)  %lu falha MAC\n",
                (unsigned long)g_frames, (unsigned long)g_retx, retx_pct,
                (unsigned long)g_mac_fail);
  Serial.printf("buffer      : %u/%u  pico %u  %s\n",
                bl_store_used(), bl_sf.n, bl_sf.peak_used,
                bl_sf.psram ? "PSRAM" : "SRAM");
  Serial.printf("perdas      : %lu records por estouro de buffer\n",
                (unsigned long)bl_sf.dropped);
  Serial.printf("            : %lu quadros abandonados por prazo (%lu records)\n",
                (unsigned long)bl_sf.giveup, (unsigned long)bl_sf.giveup_recs);
  Serial.printf("janela/RTT  : cwnd %u  srtt %.1f ms  rto %lu ms\n",
                g_cwnd, g_srtt, (unsigned long)g_rto);
  Serial.printf("RSSI do ACK : %d dBm\n", g_rssi_ack);
  Serial.printf("quedas      : %lu\n", (unsigned long)g_link_drops);
  Serial.printf("no gateway  : %lu entregues  %lu duplicatas  %lu lacunas\n",
                (unsigned long)g_peer_rx, (unsigned long)g_peer_dups,
                (unsigned long)g_peer_gaps);
#if BL_SOURCE == BL_SOURCE_UART
  Serial.printf("UART        : %lu linhas longas descartadas\n",
                (unsigned long)g_uart_overrun);
#endif
  Serial.printf("heap livre  : %lu B\n", (unsigned long)ESP.getFreeHeap());
}

static void bl_console(void) {
  static char line[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') {
      if (n == 0) continue;
      line[n] = 0;
      n = 0;

      if (!strcmp(line, "help")) {
        Serial.println(F("stats | rate <hz> | estop <0|1> | ch <1-13> | zero | help"));
      } else if (!strcmp(line, "stats")) {
        bl_print_stats();
      } else if (!strncmp(line, "rate ", 5)) {
        int hz = atoi(line + 5);
        if (hz >= 1 && hz <= 400) {
          g_src_hz = (uint16_t)hz;
          g_emu_next_us = micros();
          Serial.printf("taxa da fonte = %u Hz\n", g_src_hz);
        } else {
          Serial.println(F("faixa 1..400"));
        }
      } else if (!strncmp(line, "estop ", 6)) {
        bl_emu_set_estop((uint8_t)atoi(line + 6));
        Serial.printf("estop = %u\n", bl_emu_get_estop());
      } else if (!strncmp(line, "ch ", 3)) {
        int ch = atoi(line + 3);
        if (ch >= 1 && ch <= 13) {
          bl_radio_set_channel((uint8_t)ch);
          g_ls = LS_DISCOVER;
          Serial.printf("canal = %d, reassociando\n", ch);
        }
      } else if (!strcmp(line, "zero")) {
        g_records = g_frames = g_retx = g_mac_fail = 0;
        bl_sf.dropped = bl_sf.giveup = bl_sf.giveup_recs = 0;
        bl_sf.peak_used = 0;
        g_link_drops = 0;
        Serial.println(F("contadores zerados"));
      } else {
        Serial.println(F("? 'help'"));
      }
    } else if (n < sizeof(line) - 1) {
      line[n++] = (char)c;
    } else {
      n = 0;
    }
  }
}

// ============================================================================
//  setup / loop
// ============================================================================

void setup() {
  Serial.begin(BL_CONSOLE_BAUD);
#if ARDUINO_USB_CDC_ON_BOOT
  // Com USB CDC nativo, escrever sem host lendo bloqueia. Timeout zero faz o
  // console descartar em vez de travar o laço de controle do link.
  Serial.setTxTimeoutMs(0);
#endif
  delay(300);

  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F(" Telemetria Baja SAE - no sensor (BajaLink TX)"));
  Serial.printf (" protocolo v%d   fonte: %s\n", BL_PROTO_VER, BL_SRC_NAME);
  Serial.println(F("=============================================="));

  g_boot_id = bl_boot_id();

  if (!bl_store_init()) {
    Serial.println(F("[fatal] sem memoria para o buffer store-and-forward"));
    while (true) delay(1000);
  }
  Serial.printf("[buffer] %u slots em %s (%u KB)\n",
                bl_sf.n, bl_sf.psram ? "PSRAM" : "SRAM interna",
                (unsigned)((uint32_t)bl_sf.n * sizeof(bl_slot_t) / 1024));

  q_rx   = xQueueCreate(16, sizeof(bl_evt_t));
  sem_hw = xSemaphoreCreateCounting(BL_HW_QUEUE, BL_HW_QUEUE);
  if (!q_rx || !sem_hw) {
    Serial.println(F("[fatal] sem memoria para fila/semaforo"));
    while (true) delay(1000);
  }

#if BL_SOURCE == BL_SOURCE_UART
  // USART3 do STM32 -> UART1 deste ESP32. Só o RX é obrigatório; o TX fica
  // disponível caso se queira enviar comandos de volta para a central.
  Serial1.begin(BL_UART_BAUD, SERIAL_8N1, BL_UART_RX_PIN, BL_UART_TX_PIN);
  Serial.printf("[fonte] UART1 %lu baud  RX=GPIO%d  TX=GPIO%d\n",
                (unsigned long)BL_UART_BAUD, BL_UART_RX_PIN, BL_UART_TX_PIN);
#else
  bl_emu_init();
  g_emu_next_us = micros();
  Serial.printf("[fonte] emulador a %u Hz\n", g_src_hz);
#endif

  bl_radio_begin(g_ch_list[0]);
  if (bl_espnow_begin() != ESP_OK) {
    Serial.println(F("[fatal] esp_now_init falhou"));
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(on_espnow_recv);
  esp_now_register_send_cb(on_espnow_sent);

  Serial.printf("[radio] MAC %s  canal %u  %s  %d dBm\n",
                WiFi.macAddress().c_str(), bl_radio_get_channel(),
                bl_rate_name(), BL_TX_POWER_QDBM / 4);
  Serial.println(F("[pronto] 'help' para comandos"));

  g_ch_since   = millis();
  g_src_win_t0 = millis();
}

void loop() {
  uint32_t now = millis();

  bl_poll_source();

  // Teto de latência do empacotamento: um record não espera companhia por
  // mais de BL_AGG_MAX_LATENCY_MS.
  if (g_stage_len && (uint32_t)(now - g_stage_t0) >= BL_AGG_MAX_LATENCY_MS)
    bl_stage_flush();

  bl_service_rx(now);
  bl_service_link(now);
  bl_service_arq(now);
  bl_service_stat(now);
  bl_console();

  // Taxa medida da fonte, em janela de 1 s.
  if ((uint32_t)(now - g_src_win_t0) >= 1000) {
    g_src_hz_meas = (uint16_t)g_src_win_n;
    g_src_win_n   = 0;
    g_src_win_t0  = now;
  }
}
