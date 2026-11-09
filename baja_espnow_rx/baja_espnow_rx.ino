// ============================================================================
//  Telemetria Baja SAE — gateway de box (receptor BajaLink / ESP-NOW)
//  Elias Enns, Wesley Machado — trilha formativa de Internet das Coisas
// ----------------------------------------------------------------------------
//  Papel deste nó
//
//  Ocupa no box o lugar que o receptor LoRa STM32 ocupa hoje: recebe a
//  telemetria por rádio e a entrega ao PC como porta COM. A escolha de
//  projeto mais importante aqui é de compatibilidade — este gateway fala
//  exatamente o mesmo protocolo de linha que o receptor STM32 já falava, uma
//  linha JSON por record, de modo que o dashboard, os arquivos de calibração
//  e as regras de alerta continuam valendo sem reescrita.
//
//  O que ele acrescenta ao record
//
//    "tele"  — mantido. Se o record vier sem ele (caminho novo, em que a
//              central não sabe nada do link), o gateway o sintetiza a partir
//              dos seus próprios contadores, para que os campos que o
//              dashboard já exibia continuem preenchidos.
//    "link"  — novo. Saúde completa do caminho ESP-NOW nos dois sentidos:
//              RSSI, PDR, retransmissões, ocupação do buffer do sensor,
//              janela de congestionamento, RTT estimado, canal e taxa PHY.
//              É o que permite afirmar algo sobre a confiabilidade do link
//              em vez de supor.
//
//  Ordenação e lacunas
//
//  Com retransmissão os quadros chegam fora de ordem. A janela de
//  reordenação (bl_reorder.h) reconstitui a sequência antes de escrever na
//  porta, e declara lacuna com prazo limitado quando um quadro não vem. O log
//  resultante é monotônico no tempo e tem a perda contabilizada, não
//  disfarçada.
//
//  Placa: ESP32-S3-DevKitC-1. Ver README.md para a combinação de porta USB e
//  opção "USB CDC On Boot".
// ============================================================================

#include "bl_config.h"
#include "bl_proto.h"
#include "bl_radio.h"
#include "bl_reorder.h"
#include "bl_json.h"

// ---------------------------------------------------------------------------
// Rendezvous de canal, lado gateway.
//
// O sensor varre canais depressa (BL_CH_DWELL_MS). Se o gateway varresse na
// mesma cadência, os dois poderiam ficar em fase oposta e nunca se cruzar.
// A solução clássica é um varredor rápido e um lento: o gateway só começa a
// varrer depois de um silêncio longo, e com permanência bem maior que uma
// volta completa do sensor, o que garante sobreposição.
// ---------------------------------------------------------------------------
#define GW_SCAN_AFTER_MS   10000
#define GW_DWELL_MS        3000

// ============================================================================
//  Estado
// ============================================================================

typedef struct {
  uint8_t len;
  int8_t  rssi;
  uint8_t mac[6];
  uint8_t buf[BL_FRAME_MAX];
} bl_evt_t;

static QueueHandle_t     q_rx;
static SemaphoreHandle_t sem_hw;

static uint8_t  g_peer[6]   = {0};
static bool     g_have_peer = false;
static bool     g_assoc     = false;
static uint32_t g_boot_id   = 0;
static uint32_t g_peer_boot = 0;
static uint32_t g_last_rx_ms= 0;
static uint32_t g_link_drops= 0;
static int8_t   g_rssi_data = 0;

static const uint8_t g_ch_list[BL_CHANNEL_LIST_LEN] = BL_CHANNEL_LIST;
static uint8_t  g_ch_idx    = 0;
static uint32_t g_scan_since= 0;

// Último STAT recebido do sensor: é a metade da história que o gateway não
// consegue observar sozinho (ocupação de buffer, cwnd, RTT, descartes).
static bl_stat_t g_stat;
static bool      g_stat_valid = false;

// ACK
static uint16_t g_last_data_seq   = 0;
static uint8_t  g_since_ack       = 0;
static uint32_t g_last_ack_ms     = 0;
static uint32_t g_last_hole_ack   = 0;
static bool     g_ack_now         = false;
static uint32_t g_acks            = 0;

// PDR em janela móvel
static uint32_t g_pdr_t0 = 0, g_pdr_f0 = 0, g_pdr_g0 = 0;
static float    g_pdr    = 100.0f;

// Reassembly de record fragmentado
static uint8_t  g_frag[BL_MAX_RECORD];
static uint16_t g_fraglen = 0;
static uint32_t g_frag_err = 0;

static uint32_t g_records_out = 0;
static uint32_t g_winfull     = 0;

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
  xQueueSend(q_rx, &e, 0);
}

BL_SEND_CB(on_espnow_sent) {
  (void)status;
  xSemaphoreGive(sem_hw);
}

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
//  Saída na porta COM
// ============================================================================

// Preenche o contexto da linha de saída com o estado atual dos dois lados.
static void bl_json_fill(bl_json_ctx_t *c) {
  memset(c, 0, sizeof(*c));
  c->rx_frames  = bl_rx.frames;
  c->rx_dups    = bl_rx.dups;
  c->rx_gaps    = bl_rx.gaps;
  c->link_drops = g_link_drops;
  c->reorder    = bl_rx_depth();
  c->rssi       = g_rssi_data;
  c->pdr        = g_pdr;
  c->up         = g_assoc ? 1 : 0;
  c->channel    = bl_radio_get_channel();
  c->lr         = (uint8_t)BL_LONG_RANGE;
  if (g_stat_valid) {
    c->tx_retx    = g_stat.tx_retx;
    c->tx_dropped = g_stat.tx_dropped;
    c->tx_giveup  = g_stat.tx_giveup;
    c->buf_used   = g_stat.buf_used;
    c->buf_size   = g_stat.buf_size;
    c->cwnd       = g_stat.cwnd;
    c->srtt       = g_stat.srtt_ms;
    c->rto        = g_stat.rto_ms;
    c->src_hz     = g_stat.src_hz;
    c->lr         = g_stat.lr_mode;
    c->src        = g_stat.src;
  }
}

static void bl_emit_record(const uint8_t *rec, uint16_t len) {
  static char out[BL_JSON_OUT_CAP];
  g_records_out++;

  bl_json_ctx_t ctx;
  bl_json_fill(&ctx);

  int n = bl_json_decorate(out, sizeof(out), rec, len, &ctx);
  if (n < 0) {
    // Não parece JSON: repassa cru em vez de inventar formato.
    Serial.write(rec, len);
    Serial.write('\n');
    return;
  }
  Serial.write((const uint8_t *)out, (size_t)n);
  Serial.write('\n');
}

// Desempacota o payload de um DATA: ou é um fragmento de record longo, ou é
// um agregado de records precedidos por um byte de tamanho.
static void bl_deliver_payload(const uint8_t *p, uint16_t len, uint8_t flags) {
  if (flags & BL_F_FRAG) {
    if ((uint32_t)g_fraglen + len <= sizeof(g_frag)) {
      memcpy(g_frag + g_fraglen, p, len);
      g_fraglen = (uint16_t)(g_fraglen + len);
    } else {
      g_frag_err++;
      g_fraglen = 0;
      return;
    }
    if (!(flags & BL_F_MORE)) {
      bl_emit_record(g_frag, g_fraglen);
      g_fraglen = 0;
    }
    return;
  }

  uint16_t off = 0;
  while (off < len) {
    uint8_t rl = p[off++];
    if (rl == 0 || (uint16_t)(off + rl) > len) break;   // payload truncado
    bl_emit_record(p + off, rl);
    off = (uint16_t)(off + rl);
  }
}

// ============================================================================
//  Recepção
// ============================================================================

static void bl_service_rx(uint32_t now) {
  bl_evt_t e;
  int guard = 0;
  while (xQueueReceive(q_rx, &e, 0) == pdTRUE && guard++ < 24) {
    bl_hdr_t h;
    const uint8_t *pl;
    if (!bl_parse(e.buf, e.len, &h, &pl)) { bl_rx.bad++; continue; }

    switch (h.type) {
      case BL_T_HELLO: {
        if (h.len < sizeof(bl_hello_t)) break;
        bl_hello_t hi;
        memcpy(&hi, pl, sizeof(hi));
        if (hi.role != 0) break;                 // só o sensor emite HELLO

        bool reboot = (hi.boot_id != g_peer_boot);
        bool is_new = !g_have_peer || memcmp(g_peer, e.mac, 6) != 0 || reboot;
        memcpy(g_peer, e.mac, 6);
        g_peer_boot = hi.boot_id;
        if (is_new) {
          bl_peer_add(g_peer, BL_ENCRYPT ? true : false);
          // Reset do sensor zera a numeração: a janela precisa reancorar na
          // próxima sequência vista, senão interpretaria tudo como duplicata.
          bl_rx_unsync();
          g_stat_valid = false;
          Serial.printf("# sensor %s canal %u %s\n", bl_mac_str(g_peer),
                        bl_radio_get_channel(), bl_rate_name());
        }
        g_have_peer  = true;
        g_assoc      = true;
        g_last_rx_ms = now;

        bl_hello_t me;
        me.boot_id   = g_boot_id;
        me.proto_ver = BL_PROTO_VER;
        me.role      = 1;
        me.channel   = bl_radio_get_channel();
        me.lr_mode   = BL_LONG_RANGE;
        bl_tx_frame(BL_T_HELLO_ACK, 0, 0, &me, sizeof(me), g_peer);
        break;
      }

      case BL_T_DATA: {
        g_last_rx_ms = now;
        g_rssi_data  = e.rssi;
        g_assoc      = true;
        g_last_data_seq = h.seq;
        uint8_t r = bl_rx_store(h.seq, h.flags, pl, h.len, now);
        if (r == BL_RX_OK) g_since_ack++;
        if (h.flags & BL_F_REQACK) g_ack_now = true;
        if (r == BL_RX_DUP)        g_ack_now = true;   // o sensor não ouviu o ACK
        if (r == BL_RX_WINFULL) {
          // Janela sem espaço: o quadro não foi guardado e será retransmitido.
          // Um ACK imediato é a resposta certa, porque leva ao sensor a base
          // atual e o campo rx_free, que é o sinal de contrapressão do link.
          g_winfull++;
          g_ack_now = true;
        }
        break;
      }

      case BL_T_STAT: {
        if (h.len < sizeof(bl_stat_t)) break;
        memcpy(&g_stat, pl, sizeof(g_stat));
        g_stat_valid = true;
        g_last_rx_ms = now;
        // Encerra lacunas que o sensor já descartou: nada abaixo da base dele
        // será retransmitido, então esperar o prazo é desperdício de latência.
        g_ack_now = true;      // o catch-up abaixo usa a base recém-recebida
        break;
      }

      default: break;
    }
  }
}

// ============================================================================
//  Entrega em ordem
// ============================================================================

static void bl_service_deliver(uint32_t now) {
  static uint8_t tmp[BL_MAX_PAYLOAD];
  int guard = 0;
  bl_rslot_t *s;
  while ((s = bl_rx_head()) != NULL && guard++ < 64) {
    uint16_t l = s->len;
    uint8_t  f = s->flags;
    memcpy(tmp, s->data, l);
    bl_rx_pop(now);
    bl_deliver_payload(tmp, l, f);
  }
}

// Entrega e encerramento de lacunas mortas, alternando até não haver mais
// progresso.
//
// Por que alternar: bl_rx_forward_to para na primeira posição já ocupada,
// porque aquele quadro precisa ser entregue, não pulado. Chamar uma vez por
// quadro STAT resolvia uma lacuna a cada 250 ms. Depois de um estouro de
// buffer no sensor há dezenas de lacunas definitivas enfileiradas, e nesse
// ritmo a base do gateway andava mais devagar que a produção de dados — o
// link nunca se recuperava. Alternando entrega e avanço, todas as lacunas
// que o sensor já descartou são encerradas numa única passagem.
static void bl_service_catchup(uint32_t now) {
  if (!g_stat_valid) return;
  for (int i = 0; i < 64; i++) {
    bool progress = false;
    if (bl_rx_head()) { bl_service_deliver(now); progress = true; }
    if (!bl_rx_head() && bl_seq_gt(g_stat.tx_base_seq, bl_rx.base)) {
      if (bl_rx_forward_to(g_stat.tx_base_seq, now)) progress = true;
    }
    if (!progress) break;
  }
}

// Prazo da lacuna: só conta quando há algo adiante esperando, isto é, quando
// se sabe que um quadro específico está faltando — e não quando o sensor
// simplesmente parou de transmitir.
static void bl_service_gap(uint32_t now) {
  if (!bl_rx.synced) return;
  if (bl_rx_head()) return;
  if (!bl_rx_has_ahead()) return;
  if ((uint32_t)(now - bl_rx.t_wait) < BL_RX_HOLE_TIMEOUT_MS) return;
  bl_rx_skip(now);
  g_ack_now = true;
}

// ============================================================================
//  ACK
// ============================================================================

static void bl_service_ack(uint32_t now) {
  if (!g_have_peer) return;

  bool hole = bl_rx.synced && !bl_rx_head() && bl_rx_has_ahead();
  if (hole && (uint32_t)(now - g_last_hole_ack) < 30) hole = false;

  bool due = g_ack_now || hole
          || g_since_ack >= BL_ACK_EVERY_N
          || (g_since_ack && (uint32_t)(now - g_last_ack_ms) >= BL_ACK_MAX_DELAY_MS);
  if (!due) return;

  bl_ack_t a;
  bl_rx_build_ack(&a, g_last_data_seq, g_rssi_data);
  if (bl_tx_frame(BL_T_ACK, 0, 0, &a, sizeof(a), g_peer)) {
    g_since_ack   = 0;
    g_last_ack_ms = now;
    g_ack_now     = false;
    g_acks++;
    if (hole) g_last_hole_ack = now;
  }
}

// ============================================================================
//  Canal e associação
// ============================================================================

static void bl_service_channel(uint32_t now) {
  if (g_assoc && (uint32_t)(now - g_last_rx_ms) > BL_LINK_LOST_MS) {
    g_assoc = false;
    g_link_drops++;
    g_scan_since = now;
    // A janela de reordenação NÃO é zerada aqui de propósito: o sensor guardou
    // tudo que não foi confirmado e vai retransmitir a partir da mesma base.
    // Zerar faria o gateway descartar o que está por vir.
    Serial.printf("# link caiu (%lu)\n", (unsigned long)g_link_drops);
  }

  if (g_assoc) return;
  if ((uint32_t)(now - g_last_rx_ms) < GW_SCAN_AFTER_MS) return;
  if ((uint32_t)(now - g_scan_since) < GW_DWELL_MS) return;

  g_ch_idx = (uint8_t)((g_ch_idx + 1) % BL_CHANNEL_LIST_LEN);
  bl_radio_set_channel(g_ch_list[g_ch_idx]);
  g_scan_since = now;
  Serial.printf("# procurando sensor no canal %u\n", g_ch_list[g_ch_idx]);
}

// ============================================================================
//  PDR em janela móvel
// ============================================================================

// A taxa vitalícia esconde o que interessa: depois de dez minutos limpos, um
// buraco de trinta segundos quase não mexe na média. A janela de 1 s com
// suavização mostra o estado atual do link.
static void bl_service_pdr(uint32_t now) {
  if ((uint32_t)(now - g_pdr_t0) < 1000) return;
  uint32_t df = bl_rx.frames - g_pdr_f0;
  uint32_t dg = bl_rx.gaps   - g_pdr_g0;
  if (df + dg > 0) {
    float inst = 100.0f * (float)df / (float)(df + dg);
    g_pdr = 0.7f * g_pdr + 0.3f * inst;
  }
  g_pdr_f0 = bl_rx.frames;
  g_pdr_g0 = bl_rx.gaps;
  g_pdr_t0 = now;
}

// ============================================================================
//  Console
// ============================================================================

static void bl_print_stats(void) {
  uint32_t tot = bl_rx.frames + bl_rx.gaps;
  Serial.println(F("# ---- BajaLink gateway ----"));
  Serial.printf("# estado    : %s\n", g_assoc ? "ASSOCIADO" : "procurando");
  Serial.printf("# sensor    : %s\n", g_have_peer ? bl_mac_str(g_peer) : "-");
  Serial.printf("# canal/taxa: %u / %s\n", bl_radio_get_channel(), bl_rate_name());
  Serial.printf("# quadros   : %lu entregues  %lu duplicatas  %lu lacunas\n",
                (unsigned long)bl_rx.frames, (unsigned long)bl_rx.dups,
                (unsigned long)bl_rx.gaps);
  Serial.printf("# PDR       : %.2f%% janela   %.2f%% vitalicio\n", g_pdr,
                tot ? (100.0f * (float)bl_rx.frames / (float)tot) : 0.0f);
  Serial.printf("# records   : %lu escritos na porta\n",
                (unsigned long)g_records_out);
  Serial.printf("# reorder   : %u agora  pico %u de %u\n",
                bl_rx_depth(), bl_rx.peak_depth, (unsigned)BL_RX_WIN);
  Serial.printf("# rejeitados: %lu CRC/versao  %lu remontagem  %lu janela cheia\n",
                (unsigned long)bl_rx.bad, (unsigned long)g_frag_err,
                (unsigned long)g_winfull);
  Serial.printf("# RSSI      : %d dBm   ACKs enviados %lu\n",
                g_rssi_data, (unsigned long)g_acks);
  Serial.printf("# quedas    : %lu\n", (unsigned long)g_link_drops);
  if (g_stat_valid) {
    Serial.printf("# no sensor : buffer %u/%u  cwnd %u  srtt %u ms  rto %u ms\n",
                  g_stat.buf_used, g_stat.buf_size, g_stat.cwnd,
                  g_stat.srtt_ms, g_stat.rto_ms);
    Serial.printf("#           : %lu records  %lu quadros  %lu retx  %lu descartes\n",
                  (unsigned long)g_stat.tx_records, (unsigned long)g_stat.tx_frames,
                  (unsigned long)g_stat.tx_retx, (unsigned long)g_stat.tx_dropped);
    Serial.printf("#           : fonte %s a %u Hz  buffer em %s\n",
                  g_stat.src ? "UART" : "emulador", g_stat.src_hz,
                  g_stat.psram ? "PSRAM" : "SRAM");
  }
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
        Serial.println(F("# stats | zero | ch <1-13> | help"));
      } else if (!strcmp(line, "stats")) {
        bl_print_stats();
      } else if (!strcmp(line, "zero")) {
        bl_rx.frames = bl_rx.dups = bl_rx.gaps = bl_rx.bad = 0;
        bl_rx.peak_depth = 0;
        g_records_out = g_acks = g_link_drops = g_frag_err = g_winfull = 0;
        g_pdr = 100.0f;
        Serial.println(F("# contadores zerados"));
      } else if (!strncmp(line, "ch ", 3)) {
        int ch = atoi(line + 3);
        if (ch >= 1 && ch <= 13) {
          bl_radio_set_channel((uint8_t)ch);
          Serial.printf("# canal = %d\n", ch);
        }
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
  // Essencial aqui: sem host lendo a porta, uma escrita em CDC nativo
  // bloqueia. Se o dashboard não estiver aberto, o gateway travaria e
  // pararia de mandar ACK, derrubando o link por um motivo que nada tem a
  // ver com rádio.
  Serial.setTxTimeoutMs(0);
#endif
  delay(300);

  Serial.println();
  Serial.println(F("# ================================================"));
  Serial.println(F("#  Telemetria Baja SAE - gateway (BajaLink RX)"));
  Serial.printf ("#  protocolo v%d  saida: 1 record JSON por linha\n", BL_PROTO_VER);
  Serial.println(F("# ================================================"));

  g_boot_id = bl_boot_id();
  bl_rx_init();
  memset(&g_stat, 0, sizeof(g_stat));

  q_rx   = xQueueCreate(24, sizeof(bl_evt_t));
  sem_hw = xSemaphoreCreateCounting(BL_HW_QUEUE, BL_HW_QUEUE);
  if (!q_rx || !sem_hw) {
    Serial.println(F("# [fatal] sem memoria para fila/semaforo"));
    while (true) delay(1000);
  }

  bl_radio_begin(BL_CHANNEL_HOME);
  if (bl_espnow_begin() != ESP_OK) {
    Serial.println(F("# [fatal] esp_now_init falhou"));
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(on_espnow_recv);
  esp_now_register_send_cb(on_espnow_sent);

  // Procura o canal de casa a partir da posição correspondente na lista, para
  // que a varredura, se ocorrer, comece do vizinho certo.
  for (uint8_t i = 0; i < BL_CHANNEL_LIST_LEN; i++)
    if (g_ch_list[i] == BL_CHANNEL_HOME) g_ch_idx = i;

  Serial.printf("# MAC %s  canal %u  %s  %d dBm\n",
                WiFi.macAddress().c_str(), bl_radio_get_channel(),
                bl_rate_name(), BL_TX_POWER_QDBM / 4);
  Serial.println(F("# aguardando sensor"));

  uint32_t now = millis();
  g_last_rx_ms = now;
  g_scan_since = now;
  g_pdr_t0     = now;
}

void loop() {
  uint32_t now = millis();
  bl_service_rx(now);
  bl_service_deliver(now);
  bl_service_catchup(now);
  bl_service_gap(now);
  bl_service_ack(now);
  bl_service_channel(now);
  bl_service_pdr(now);
  bl_console();
}
