// ============================================================================
//  Teste de integração do BajaLink rodando no host.
//
//  Exercita as estruturas reais (bl_proto, bl_store, bl_reorder, bl_emu)
//  contra um canal com perda e reordenação sintéticas, e verifica as
//  propriedades que o link promete:
//
//    1. Todo record entregue está na ordem em que foi produzido.
//    2. Nenhum record é entregue duas vezes.
//    3. Com perda moderada e buffer suficiente, nada se perde.
//    4. Quando o buffer estoura, a perda é contabilizada, não silenciosa.
//    5. A aritmética de sequência sobrevive ao wrap de 16 bits.
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <deque>
#include <cstdint>
#include <cmath>

#include "bl_config.h"
#include "bl_proto.h"
#include "bl_store.h"
#include "bl_reorder.h"
#include "bl_emu.h"

// ---------------------------------------------------------------------------
// Infra de teste
// ---------------------------------------------------------------------------
static int g_fail = 0;
static void check(bool ok, const char *what) {
  printf("  [%s] %s\n", ok ? "ok" : "FALHOU", what);
  if (!ok) g_fail++;
}

static uint32_t g_now = 0;                    // "millis()" virtual
static uint32_t rnd_state = 12345;
static uint32_t rnd(void) {
  rnd_state = rnd_state * 1664525u + 1013904223u;
  return rnd_state >> 8;
}
static bool chance(int pct) { return (int)(rnd() % 100) < pct; }

// ---------------------------------------------------------------------------
// Canal: entrega com atraso, perda e reordenação
// ---------------------------------------------------------------------------
struct Pkt { uint32_t at; std::vector<uint8_t> b; };
static std::deque<Pkt> ch_fwd, ch_rev;

static void ch_send(std::deque<Pkt> &ch, const uint8_t *b, size_t n,
                    int loss_pct, uint32_t base_delay) {
  if (chance(loss_pct)) return;
  uint32_t jitter = rnd() % 12;               // produz reordenação de verdade
  ch.push_back({ g_now + base_delay + jitter, std::vector<uint8_t>(b, b + n) });
}

static bool ch_recv(std::deque<Pkt> &ch, std::vector<uint8_t> &out) {
  for (size_t i = 0; i < ch.size(); i++) {
    if (ch[i].at <= g_now) {
      out = ch[i].b;
      ch.erase(ch.begin() + i);
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Lado TX: empacotamento + ARQ (mesma lógica do sketch, condensada)
// ---------------------------------------------------------------------------
static uint8_t  stage[BL_MAX_PAYLOAD];
static uint16_t stage_len = 0, stage_recs = 0;
static uint16_t cwnd = BL_CWND_INIT;
static uint16_t acked_run = 0;
static uint32_t rto = 80;
static uint32_t tx_retx = 0;
static uint16_t recover_seq = 0;
static bool     in_recovery = false;
static float    srtt = 0.0f, rttvar = 0.0f;
static int      g_loss = 0;      // perda corrente do canal, usada pelo retx rápido

static void tx_on_loss(void);
static void tx_arq_up(int loss_pct);

// Mesmo estimador do firmware (RFC 6298). Sem ele o RTO fica muito acima do
// RTT real, e como a janela limita quadros dentro do RTO, a vazao do link
// cai para cwnd/RTO em vez de cwnd/RTT. Era o que fazia o cenario de 30 %
// de perda estourar o buffer no teste.
static void tx_rtt_update(uint32_t sample) {
  if (sample > BL_RTO_MAX_MS * 4) return;
  if (srtt <= 0.0f) { srtt = (float)sample; rttvar = (float)sample / 2.0f; }
  else {
    rttvar = 0.75f * rttvar + 0.25f * fabsf(srtt - (float)sample);
    srtt   = 0.875f * srtt + 0.125f * (float)sample;
  }
  float r = srtt + 4.0f * rttvar;
  float floor_mult = srtt * 1.25f;
  if (r < floor_mult) r = floor_mult;
  r += (float)BL_RTO_SLACK_MS;
  if (r < BL_RTO_MIN_MS) r = BL_RTO_MIN_MS;
  if (r > BL_RTO_MAX_MS) r = BL_RTO_MAX_MS;
  rto = (uint32_t)r;
}

static void stage_flush(void) {
  if (!stage_len) return;
  bl_store_push(stage, stage_len, (uint8_t)stage_recs, 0, NULL);
  stage_len = stage_recs = 0;
}

static void tx_push_record(const char *rec, uint16_t len) {
  if ((uint16_t)(stage_len + 1 + len) > BL_MAX_PAYLOAD) stage_flush();
  stage[stage_len++] = (uint8_t)len;
  memcpy(stage + stage_len, rec, len);
  stage_len = (uint16_t)(stage_len + len);
  stage_recs++;
  if (stage_recs >= BL_AGG_MAX_RECORDS) stage_flush();
}

// Estado do link, como em bl_service_link() no firmware: com o link caido o
// sensor nao transmite DATA, apenas sonda. E o que impede uma queda de rede de
// queimar as BL_MAX_TRIES de quadros que o buffer deveria estar apenas
// guardando.
static uint32_t last_ack_rx = 0;
static bool     link_up = true;
static uint32_t last_probe = 0;

static void tx_arq(int loss_pct) {
  if ((uint32_t)(g_now - last_ack_rx) > BL_LINK_LOST_MS) {
    if (link_up) { link_up = false; cwnd = BL_CWND_INIT; in_recovery = false; }
  }
  if (!link_up) {
    // Sonda periodica no lugar do HELLO: barata e suficiente para detectar
    // o retorno do link sem gastar tentativas dos quadros enfileirados.
    if ((uint32_t)(g_now - last_probe) >= BL_HELLO_PERIOD_MS) {
      last_probe = g_now;
      bl_slot_t *h = bl_store_get(bl_sf.base_seq);
      if (h) {
        uint8_t fr[BL_FRAME_MAX];
        size_t n = bl_build(fr, BL_T_DATA, (uint8_t)(h->flags | BL_F_REQACK),
                            bl_sf.base_seq, h->data, h->len);
        ch_send(ch_fwd, fr, n, loss_pct, 3);
        if (h->tries == 0) { h->tries = 1; h->t_first_ms = h->t_last_ms = g_now; }
        else h->t_last_ms = g_now;
      }
    }
    return;
  }
  tx_arq_up(loss_pct);
}

static void tx_arq_up(int loss_pct) {
  // Limpa a cabeça: slots liberados por SACK e quadros esgotados.
  for (;;) {
    if (bl_store_used() == 0) break;
    bl_slot_t *h = &bl_sf.slots[bl_sf.base_seq % bl_sf.n];
    if (!h->used) { bl_sf.base_seq++; continue; }
    if (h->tries >= BL_MAX_TRIES && (uint32_t)(g_now - h->t_last_ms) >= rto) {
      bl_store_giveup_head();
      continue;
    }
    break;
  }

  uint16_t used = bl_store_used();
  uint16_t inflight = 0;
  uint8_t frame[BL_FRAME_MAX];

  for (uint16_t i = 0; i < used; i++) {
    uint16_t seq = (uint16_t)(bl_sf.base_seq + i);
    bl_slot_t *s = bl_store_get(seq);
    if (!s || s->tries == 0) continue;
    uint8_t shift = (uint8_t)((s->tries > 5) ? 4 : (s->tries - 1));
    uint32_t r = rto << shift;
    if (r > BL_RTO_MAX_MS) r = BL_RTO_MAX_MS;
    if ((uint32_t)(g_now - s->t_last_ms) < r) { inflight++; continue; }
    if (s->tries >= BL_MAX_TRIES) continue;
    size_t n = bl_build(frame, BL_T_DATA, (uint8_t)(s->flags | BL_F_RETX),
                        seq, s->data, s->len);
    ch_send(ch_fwd, frame, n, loss_pct, 3);
    s->tries++;
    s->t_last_ms = g_now;
    tx_retx++;
    inflight++;
    tx_on_loss();
  }

  for (uint16_t i = 0; i < used && inflight < cwnd; i++) {
    uint16_t seq = (uint16_t)(bl_sf.base_seq + i);
    bl_slot_t *s = bl_store_get(seq);
    if (!s || s->tries != 0) continue;
    size_t n = bl_build(frame, BL_T_DATA, s->flags, seq, s->data, s->len);
    ch_send(ch_fwd, frame, n, loss_pct, 3);
    s->tries = 1;
    s->t_first_ms = s->t_last_ms = g_now;
    inflight++;
  }
}

// Época de recuperação: uma redução por evento de perda, como no firmware.
static void tx_on_loss(void) {
  if (in_recovery) return;
  in_recovery = true;
  recover_seq = bl_sf.next_seq;
  if ((uint32_t)bl_store_used() * 2 >= (uint32_t)bl_sf.n) { acked_run = 0; return; }
  cwnd = (uint16_t)(cwnd / 2);
  if (cwnd < BL_CWND_MIN) cwnd = BL_CWND_MIN;
  acked_run = 0;
}

static void tx_handle_ack(const bl_ack_t *a) {
  bl_slot_t *echoed = bl_store_get(a->echo_seq);
  if (echoed && echoed->tries == 1 && echoed->t_last_ms)
    tx_rtt_update((uint32_t)(g_now - echoed->t_last_ms));

  uint16_t freed = bl_store_ack_upto(a->ack_base);
  if (in_recovery && bl_seq_ge(bl_sf.base_seq, recover_seq)) in_recovery = false;
  if (freed && !in_recovery) {
    acked_run = (uint16_t)(acked_run + freed);
    if (acked_run >= cwnd) {
      acked_run = 0;
      if (cwnd < BL_CWND_MAX) cwnd++;
    }
  }
  if (a->sack_bits) {
    for (uint8_t i = 0; i < 32; i++)
      if (a->sack_bits & (1u << i))
        bl_store_ack_one((uint16_t)(a->ack_base + 1 + i));

    // Retransmissão rápida: há bits acima da base, logo a base é uma lacuna
    // confirmada. É esta parte que impede um buraco de travar o pipeline
    // inteiro até o RTO expirar.
    uint32_t min_gap = (uint32_t)srtt;
    if (min_gap < BL_RTO_MIN_MS) min_gap = BL_RTO_MIN_MS;
    bl_slot_t *hole = bl_store_get(a->ack_base);
    if (hole && hole->tries > 0 && hole->tries < BL_MAX_TRIES &&
        (uint32_t)(g_now - hole->t_last_ms) >= min_gap) {
      uint8_t frame[BL_FRAME_MAX];
      size_t n = bl_build(frame, BL_T_DATA, (uint8_t)(hole->flags | BL_F_RETX),
                          a->ack_base, hole->data, hole->len);
      ch_send(ch_fwd, frame, n, g_loss, 3);
      hole->tries++;
      hole->t_last_ms = g_now;
      tx_retx++;
      tx_on_loss();
    }
  }
}

// ---------------------------------------------------------------------------
// Lado RX: entrega em ordem, com verificação de monotonicidade
// ---------------------------------------------------------------------------
static std::vector<std::string> delivered;
static uint16_t rx_tx_base = 0;
static bool     rx_tx_base_valid = false;

static void rx_emit(const uint8_t *p, uint16_t len) {
  delivered.push_back(std::string((const char *)p, len));
}

static void rx_deliver_payload(const uint8_t *p, uint16_t len) {
  uint16_t off = 0;
  while (off < len) {
    uint8_t rl = p[off++];
    if (rl == 0 || (uint16_t)(off + rl) > len) break;
    rx_emit(p + off, rl);
    off = (uint16_t)(off + rl);
  }
}

static void rx_pump(int loss_pct) {
  std::vector<uint8_t> in;
  uint8_t pend_ack = 0;
  uint16_t echo = 0;

  while (ch_recv(ch_fwd, in)) {
    bl_hdr_t h;
    const uint8_t *pl;
    if (!bl_parse(in.data(), in.size(), &h, &pl)) continue;
    if (h.type == BL_T_STAT) {
      bl_stat_t s;
      if (h.len >= sizeof(s)) {
        memcpy(&s, pl, sizeof(s));
        rx_tx_base = s.tx_base_seq;
        rx_tx_base_valid = true;
        pend_ack++;
      }
      continue;
    }
    if (h.type != BL_T_DATA) continue;
    echo = h.seq;
    uint8_t r = bl_rx_store(h.seq, h.flags, pl, h.len, g_now);
    if (r == BL_RX_OK || r == BL_RX_WINFULL) pend_ack++;
  }

  // Entrega em ordem, alternada com o encerramento de lacunas que o sensor
  // ja descartou (mesma logica de bl_service_catchup no gateway).
  bl_rslot_t *s;
  uint8_t tmp[BL_MAX_PAYLOAD];
  for (int it = 0; it < 64; it++) {
    bool progress = false;
    while ((s = bl_rx_head()) != NULL) {
      uint16_t l = s->len;
      memcpy(tmp, s->data, l);
      bl_rx_pop(g_now);
      rx_deliver_payload(tmp, l);
      progress = true;
    }
    if (rx_tx_base_valid && !bl_rx_head() &&
        bl_seq_gt(rx_tx_base, bl_rx.base)) {
      if (bl_rx_forward_to(rx_tx_base, g_now)) progress = true;
    }
    if (!progress) break;
  }

  // Prazo da lacuna
  if (bl_rx.synced && !bl_rx_head() && bl_rx_has_ahead() &&
      (uint32_t)(g_now - bl_rx.t_wait) >= BL_RX_HOLE_TIMEOUT_MS) {
    bl_rx_skip(g_now);
    pend_ack++;
  }

  if (pend_ack) {
    bl_ack_t a;
    bl_rx_build_ack(&a, echo, -70);
    uint8_t frame[BL_FRAME_MAX];
    size_t n = bl_build(frame, BL_T_ACK, 0, 0, &a, sizeof(a));
    ch_send(ch_rev, frame, n, loss_pct, 3);
  }
}

// STAT a cada 500 ms, como o firmware. Carrega tx_base_seq, que permite ao
// gateway encerrar na hora as lacunas que o sensor ja descartou.
static void tx_send_stat(int loss_pct) {
  static uint32_t last = 0;
  if ((uint32_t)(g_now - last) < BL_STAT_PERIOD_MS) return;
  last = g_now;
  bl_stat_t s;
  memset(&s, 0, sizeof(s));
  s.tx_base_seq = bl_sf.base_seq;
  s.buf_used = bl_store_used();
  s.buf_size = bl_sf.n;
  s.cwnd = cwnd;
  uint8_t frame[BL_FRAME_MAX];
  size_t n = bl_build(frame, BL_T_STAT, 0, 0, &s, sizeof(s));
  ch_send(ch_fwd, frame, n, loss_pct, 3);
}

static void tx_pump_acks(void) {
  std::vector<uint8_t> in;
  while (ch_recv(ch_rev, in)) {
    bl_hdr_t h;
    const uint8_t *pl;
    if (!bl_parse(in.data(), in.size(), &h, &pl)) continue;
    if (h.type != BL_T_ACK || h.len < sizeof(bl_ack_t)) continue;
    last_ack_rx = g_now;
    if (!link_up) link_up = true;
    bl_ack_t a;
    memcpy(&a, pl, sizeof(a));
    tx_handle_ack(&a);
  }
}

// ---------------------------------------------------------------------------
// Cenário completo
// ---------------------------------------------------------------------------
struct Result { int produced; int delivered; bool ordered; int dups; };

static Result run_scenario(const char *name, int n_records, int loss_pct,
                           int outage_from, int outage_to) {
  printf("\n%s (perda %d%%%s)\n", name, loss_pct,
         outage_to > outage_from ? ", com apagão" : "");
  g_now = 0;
  rnd_state = 999;
  ch_fwd.clear();
  ch_rev.clear();
  delivered.clear();
  rx_tx_base = 0;
  rx_tx_base_valid = false;
  stage_len = stage_recs = 0;
  cwnd = BL_CWND_INIT;
  acked_run = 0;
  in_recovery = false;
  recover_seq = 0;
  srtt = rttvar = 0.0f;
  rto = 80;
  last_ack_rx = 0;
  last_probe = 0;
  link_up = true;
  tx_retx = 0;
  bl_store_init();
  bl_rx_init();
  bl_emu_init();

  std::vector<std::string> produced;
  int produced_n = 0;

  // 20 ms por passo, um record por passo (50 Hz)
  for (int step = 0; step < n_records + 4000; step++) {
    g_now += 20;
    bool blackout = (step >= outage_from && step < outage_to);

    if (step < n_records) {
      bl_emu_step(0.02f);
      char rec[BL_MAX_RECORD];
      int n = bl_emu_format(rec, sizeof(rec), g_now, (uint32_t)(++produced_n));
      produced.push_back(std::string(rec, n));
      tx_push_record(rec, (uint16_t)n);
    }
    if (stage_len && step % 1 == 0) stage_flush();

    int lp = blackout ? 100 : loss_pct;
    g_loss = lp;
    tx_arq(lp);
    tx_send_stat(lp);
    rx_pump(lp);
    tx_pump_acks();

#ifdef BL_TRACE
    if (step % 200 == 0 && step < n_records)
      printf("    t=%6u step=%4d used=%3u base=%5u next=%5u cwnd=%2u "
             "rxbase=%5u rxdepth=%3u gaps=%4lu drop=%4lu retx=%4lu\n",
             g_now, step, bl_store_used(), bl_sf.base_seq, bl_sf.next_seq, cwnd,
             bl_rx.base, bl_rx_depth(), (unsigned long)bl_rx.gaps,
             (unsigned long)bl_sf.dropped, (unsigned long)tx_retx);
#endif
    if (step >= n_records && bl_store_used() == 0 && stage_len == 0 &&
        ch_fwd.empty() && ch_rev.empty() &&
        bl_rx_head() == NULL && bl_rx_depth() == 0)
      break;
  }

  // Verifica ordem e duplicação contra a sequência produzida
  bool ordered = true;
  int dups = 0;
  size_t pi = 0;
  for (size_t i = 0; i < delivered.size(); i++) {
    if (i > 0 && delivered[i] == delivered[i - 1]) dups++;
    while (pi < produced.size() && produced[pi] != delivered[i]) pi++;
    if (pi >= produced.size()) { ordered = false; break; }
    pi++;
  }

  printf("  produzidos %d, entregues %zu, retx %lu, lacunas %lu, descartes %lu\n",
         produced_n, delivered.size(), (unsigned long)tx_retx,
         (unsigned long)bl_rx.gaps, (unsigned long)bl_sf.dropped);

  return { produced_n, (int)delivered.size(), ordered, dups };
}

// ---------------------------------------------------------------------------
int main(void) {
  printf("=== BajaLink: teste de integração ===\n");

  printf("\nformato de fio\n");
  {
    uint8_t f[BL_FRAME_MAX];
    const char *msg = "{\"ts\":1,\"seq\":1}";
    size_t n = bl_build(f, BL_T_DATA, BL_F_REQACK, 0x1234,
                        msg, (uint16_t)strlen(msg));
    check(n == BL_HDR_LEN + strlen(msg) + BL_CRC_LEN, "tamanho do quadro");
    bl_hdr_t h; const uint8_t *pl;
    check(bl_parse(f, n, &h, &pl), "parse do quadro válido");
    check(h.seq == 0x1234 && h.flags == BL_F_REQACK, "cabeçalho preservado");
    check(memcmp(pl, msg, strlen(msg)) == 0, "payload preservado");
    f[12] ^= 0x01;
    check(!bl_parse(f, n, &h, &pl), "CRC rejeita bit invertido");
    f[12] ^= 0x01;
    f[1] = 99;
    check(!bl_parse(f, n, &h, &pl), "versão divergente rejeitada");
    check(bl_build(f, BL_T_DATA, 0, 0, msg, BL_MAX_PAYLOAD + 1) == 0,
          "payload grande demais recusado");
  }

  printf("\naritmética de sequência no wrap de 16 bits\n");
  {
    check(bl_seq_gt(0, 65535), "0 > 65535");
    check(bl_seq_gt(5, 65530), "5 > 65530");
    check(!bl_seq_gt(65530, 5), "65530 nao > 5");
    check(bl_seq_diff(3, 65534) == 5, "diferenca atravessa o wrap");
  }

  printf("\njanela de reordenação\n");
  {
    bl_rx_init();
    const char *r = "\x05" "abcde";
    bl_rx_store(1000, 0, (const uint8_t *)r, 6, 0);
    check(bl_rx_head() != NULL, "ancora a base no primeiro quadro");
    bl_rx_pop(0);
    bl_rx_store(1002, 0, (const uint8_t *)r, 6, 0);
    check(bl_rx_head() == NULL, "fora de ordem nao entrega na frente");
    check(bl_rx_has_ahead(), "detecta quadro adiante");
    bl_rx_store(1001, 0, (const uint8_t *)r, 6, 0);
    check(bl_rx_head() != NULL, "lacuna preenchida libera a sequencia");
    bl_rx_pop(0); bl_rx_pop(0);
    uint32_t before = bl_rx.dups;
    bl_rx_store(1000, 0, (const uint8_t *)r, 6, 0);
    check(bl_rx.dups == before + 1, "retransmissao antiga conta como duplicata");

    // Encerramento antecipado via base informada pelo sensor
    bl_rx_init();
    bl_rx_store(500, 0, (const uint8_t *)r, 6, 0);
    bl_rx_pop(0);
    bl_rx_store(510, 0, (const uint8_t *)r, 6, 0);
    uint16_t sk = bl_rx_forward_to(510, 0);
    check(sk == 9, "forward_to declara as 9 lacunas de uma vez");
    check(bl_rx_head() != NULL, "e entrega o quadro que ja havia chegado");
  }

  printf("\nbuffer store-and-forward\n");
  {
    bl_store_init();
    check(bl_sf.n >= 32, "profundidade alocada");
    uint8_t p[16] = {0};
    for (uint16_t i = 0; i < bl_sf.n; i++) bl_store_push(p, 16, 1, 0, NULL);
    check(bl_store_free() == 0, "enche exatamente a profundidade");
    uint32_t dropped = bl_sf.dropped;
    bl_store_push(p, 16, 1, 0, NULL);
    check(bl_sf.dropped == dropped + 1, "estouro contabiliza o descarte");
    check(bl_store_used() == bl_sf.n, "ocupacao continua no limite");
  }

  Result a = run_scenario("cenário 1: link limpo", 3000, 0, 0, 0);
  check(a.delivered == a.produced, "entrega completa");
  check(a.ordered, "ordem preservada");
  check(a.dups == 0, "sem duplicatas");

  // A 15 % de perda a retransmissao recupera praticamente tudo. O residuo nao
  // e defeito, e o preco explicito do teto de latencia: um quadro que falha
  // quatro vezes seguidas ultrapassa BL_RX_HOLE_TIMEOUT_MS, e o gateway
  // prefere declarar a lacuna a congelar a tela esperando por ele. Com
  // p = 0,15 isso acontece em cerca de 5 em 10 mil quadros.
  Result b = run_scenario("cenário 2: 15% de perda", 3000, 15, 0, 0);
  check(b.delivered >= (int)(b.produced * 0.995), "entrega >= 99,5% a 15% de perda");
  check(b.ordered, "ordem preservada sob perda");
  check(b.dups == 0, "sem duplicatas sob perda");
  check(b.produced - b.delivered
          <= (int)(bl_sf.dropped + bl_sf.giveup_recs + bl_rx.gaps),
        "residuo contabilizado, nao silencioso");

  // A 30 % de perda de quadro o canal esta em frangalhos. A exigencia aqui
  // nao e entrega integral, e sim que a degradacao seja graciosa e honesta:
  // a maior parte dos dados passa, a ordem se mantem, e tudo que se perdeu
  // aparece nos contadores em vez de desaparecer em silencio.
  Result c = run_scenario("cenário 3: 30% de perda", 3000, 30, 0, 0);
  check(c.delivered >= (int)(c.produced * 0.90), "entrega >= 90% a 30% de perda");
  check(c.ordered, "ordem preservada a 30%");
  check(c.dups == 0, "sem duplicatas a 30%");
  check(c.produced - c.delivered
          <= (int)(bl_sf.dropped + bl_sf.giveup_recs + bl_rx.gaps),
        "toda perda esta contabilizada");

  // Apagão de 2 s (100 passos) com buffer de BL_SLOTS_INTERNAL slots.
  // A propriedade que interessa aqui e especifica: o apagao de 2 s nao deve
  // causar NENHUM estouro de buffer. O residuo de entrega vem do 5 % de perda
  // de fundo e do teto de latencia, nao do apagao.
  Result d = run_scenario("cenário 4: apagão de 2 s", 3000, 5, 800, 900);
  check(d.ordered, "ordem preservada apos o apagao");
  check(bl_sf.dropped == 0, "apagao de 2 s nao estoura o buffer");
  check(d.delivered >= (int)(d.produced * 0.995), "entrega >= 99,5% com apagao");

  // Apagão longo o bastante para estourar o buffer: aqui a perda é esperada,
  // mas tem de estar contabilizada.
  Result e = run_scenario("cenário 5: apagão de 30 s (estoura o buffer)",
                          3000, 5, 600, 2100);
  check(e.ordered, "ordem preservada apos estouro");
  check(e.delivered < e.produced, "perda ocorre, como esperado");
  check(bl_sf.dropped > 0, "perda aparece como estouro de buffer no sensor");
  check(bl_rx.gaps > 0, "e como lacuna declarada no gateway");
  printf("  perdidos %d records; contador de descarte do sensor: %lu\n",
         e.produced - e.delivered, (unsigned long)bl_sf.dropped);

  printf("\nemulador\n");
  {
    bl_emu_init();
    char rec[BL_MAX_RECORD];
    int maxlen = 0;
    bool has_all = true;
    for (int i = 0; i < 4000; i++) {
      bl_emu_step(0.02f);
      int n = bl_emu_format(rec, sizeof(rec), (uint32_t)i * 20, (uint32_t)i);
      if (n > maxlen) maxlen = n;
      if (!strstr(rec, "\"motor_rpm\"") || !strstr(rec, "\"cvt_temp\"") ||
          !strstr(rec, "\"estop\"")) has_all = false;
    }
    printf("  record maior: %d B (cabe em %d de payload)\n", maxlen, BL_MAX_PAYLOAD);
    check(has_all, "campos do esquema presentes");
    check(maxlen <= BL_REC_MAX_INLINE, "record cabe num quadro sem fragmentar");
  }

  printf("\n=== %s ===\n", g_fail == 0 ? "todos os testes passaram"
                                       : "HOUVE FALHAS");
  return g_fail ? 1 : 0;
}
