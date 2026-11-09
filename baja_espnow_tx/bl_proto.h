// ============================================================================
//  BajaLink — formato de fio e estruturas de controle
// ----------------------------------------------------------------------------
//  ARQUIVO COMPARTILHADO. Mantenha byte-a-byte idêntico em:
//      baja_espnow_tx/bl_proto.h
//      baja_espnow_rx/bl_proto.h
//
//  Camada de transporte sobre ESP-NOW. O ESP-NOW já entrega integridade de
//  quadro (FCS) e, opcionalmente, confidencialidade (CCMP), mas não entrega
//  nada do que um datalogger precisa: ordenação, detecção de lacuna,
//  retransmissão e armazenamento do que não foi aceito. É isso que esta
//  camada acrescenta.
//
//  Quadro completo (máx. 250 B, limite do ESP-NOW v1):
//
//      +--------+--------+--------+--------+--------+--------+--------+
//      | magic  |  ver   |  type  | flags  |      seq        |  len   |
//      |  1 B   |  1 B   |  1 B   |  1 B   |      2 B        |  2 B   |
//      +--------+--------+--------+--------+--------+--------+--------+
//      |                      payload (0..240 B)                     |
//      +-------------------------------------------------------------+
//      |                   CRC16-CCITT (2 B, LE)                     |
//      +-------------------------------------------------------------+
//
//  O CRC é redundante em relação ao FCS do 802.11 de propósito: ele cobre o
//  caminho inteiro, incluindo o salto UART entre o STM32 e o ESP32, onde um
//  byte perdido por overrun não seria detectado por nada mais.
// ============================================================================
#ifndef BL_PROTO_H
#define BL_PROTO_H

#include <stdint.h>
#include <string.h>
#include "bl_config.h"

#define BL_MAGIC                0xB7
#define BL_HDR_LEN              8
#define BL_CRC_LEN              2
#define BL_FRAME_MAX            (BL_HDR_LEN + BL_MAX_PAYLOAD + BL_CRC_LEN)  // 250

// ---------------------------------------------------------------------------
// Tipos de quadro
// ---------------------------------------------------------------------------
enum : uint8_t {
  BL_T_DATA      = 0x01,  // sensor -> gateway: records de telemetria
  BL_T_ACK       = 0x02,  // gateway -> sensor: ACK cumulativo + SACK
  BL_T_STAT      = 0x03,  // sensor -> gateway: saúde do lado transmissor
  BL_T_HELLO     = 0x04,  // sensor -> broadcast: descoberta
  BL_T_HELLO_ACK = 0x05,  // gateway -> sensor: resposta unicast
};

// ---------------------------------------------------------------------------
// Flags
// ---------------------------------------------------------------------------
#define BL_F_MORE    0x01  // há continuação deste record no próximo frame
#define BL_F_FRAG    0x02  // payload é fragmento de um único record
#define BL_F_RETX    0x04  // retransmissão (não conta como novo no PDR)
#define BL_F_GAP     0x08  // houve descarte por estouro de buffer antes daqui
#define BL_F_REQACK  0x10  // peço ACK imediato, não espere o agrupamento

// ---------------------------------------------------------------------------
// Cabeçalho
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  ver;
  uint8_t  type;
  uint8_t  flags;
  uint16_t seq;
  uint16_t len;
} bl_hdr_t;

// ---------------------------------------------------------------------------
// ACK cumulativo + seletivo
//
// ack_base é o próximo seq esperado: tudo abaixo dele já foi entregue em
// ordem e pode ser liberado do buffer. sack_bits descreve os 32 seq a partir
// de ack_base que chegaram fora de ordem — o bit i corresponde a
// ack_base + 1 + i. Um bit zerado com bits mais altos acesos é uma lacuna
// confirmada, e dispara retransmissão rápida sem esperar o RTO.
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint16_t ack_base;
  uint32_t sack_bits;
  uint16_t echo_seq;    // seq do DATA que provocou este ACK (medida de RTT)
  uint32_t rx_frames;   // frames únicos entregues
  uint32_t rx_dups;     // duplicatas descartadas
  uint32_t rx_gaps;     // frames declarados perdidos por timeout
  int8_t   rssi;        // RSSI do último DATA, visto pelo gateway
  uint8_t  rx_free;     // slots livres na janela de reordenação
} bl_ack_t;

// ---------------------------------------------------------------------------
// Saúde do transmissor, espelhada no gateway para aparecer no dashboard.
//
// tx_base_seq fecha o laço do descarte: quando o buffer do sensor estoura, os
// seq descartados nunca vão chegar. Informando a base, o gateway encerra a
// lacuna na hora em vez de esperar BL_RX_HOLE_TIMEOUT_MS por um frame que não
// existe mais. É a mesma ideia do FORWARD-TSN do SCTP em modo parcialmente
// confiável.
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint32_t tx_records;   // records lidos da fonte
  uint32_t tx_frames;    // frames DATA originais
  uint32_t tx_retx;      // retransmissões emitidas
  uint32_t tx_dropped;   // records perdidos por estouro de buffer
  uint32_t tx_giveup;    // frames abandonados após BL_MAX_TRIES
  uint16_t tx_base_seq;  // base da janela: nada abaixo disso será reenviado
  uint16_t buf_used;
  uint16_t buf_size;
  uint16_t cwnd;
  uint16_t srtt_ms;
  uint16_t rto_ms;
  uint16_t src_hz;       // taxa medida da fonte de dados
  int8_t   rssi_ack;     // RSSI do último ACK, visto pelo sensor
  uint8_t  channel;
  uint8_t  lr_mode;      // 0=off 1=LR+legado 2=LR exclusivo
  uint8_t  src;          // 0=emulador 1=UART
  uint8_t  psram;        // 1 = buffer profundo em PSRAM
  uint32_t uptime_s;
} bl_stat_t;

// ---------------------------------------------------------------------------
// Descoberta
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint32_t boot_id;      // muda a cada boot: detecta reset do outro lado
  uint8_t  proto_ver;
  uint8_t  role;         // 0 = sensor, 1 = gateway
  uint8_t  channel;
  uint8_t  lr_mode;
} bl_hello_t;

// ---------------------------------------------------------------------------
// CRC16-CCITT (polinômio 0x1021, init 0xFFFF)
//
// Versão bit-a-bit, sem tabela: a 240 B por quadro e ~50 quadros/s custa
// menos de 0,1 % de um núcleo a 240 MHz, e economiza 512 B de RAM.
// ---------------------------------------------------------------------------
static inline uint16_t bl_crc16(const uint8_t *p, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ---------------------------------------------------------------------------
// Aritmética de número de sequência (RFC 1982): comparação por diferença
// com sinal, para sobreviver ao wrap de 16 bits.
// ---------------------------------------------------------------------------
static inline bool bl_seq_gt(uint16_t a, uint16_t b) {
  return (int16_t)(a - b) > 0;
}
static inline bool bl_seq_ge(uint16_t a, uint16_t b) {
  return (int16_t)(a - b) >= 0;
}
static inline int32_t bl_seq_diff(uint16_t a, uint16_t b) {
  return (int32_t)(int16_t)(a - b);
}

// ---------------------------------------------------------------------------
// Montagem e validação de quadro
// ---------------------------------------------------------------------------

// Retorna o tamanho total do quadro, ou 0 se o payload não couber.
static inline size_t bl_build(uint8_t *out, uint8_t type, uint8_t flags,
                              uint16_t seq, const void *payload, uint16_t len) {
  if (len > BL_MAX_PAYLOAD) return 0;
  bl_hdr_t *h = (bl_hdr_t *)out;
  h->magic = BL_MAGIC;
  h->ver   = BL_PROTO_VER;
  h->type  = type;
  h->flags = flags;
  h->seq   = seq;
  h->len   = len;
  if (len && payload) memcpy(out + BL_HDR_LEN, payload, len);
  uint16_t crc = bl_crc16(out, BL_HDR_LEN + len);
  out[BL_HDR_LEN + len]     = (uint8_t)(crc & 0xFF);
  out[BL_HDR_LEN + len + 1] = (uint8_t)(crc >> 8);
  return BL_HDR_LEN + len + BL_CRC_LEN;
}

// Valida magic, versão, coerência de tamanho e CRC. Em caso de sucesso
// devolve o cabeçalho e um ponteiro para o payload dentro do buffer original.
static inline bool bl_parse(const uint8_t *in, size_t n, bl_hdr_t *hdr,
                            const uint8_t **payload) {
  if (n < BL_HDR_LEN + BL_CRC_LEN) return false;
  const bl_hdr_t *h = (const bl_hdr_t *)in;
  if (h->magic != BL_MAGIC) return false;
  if (h->ver != BL_PROTO_VER) return false;
  if (h->len > BL_MAX_PAYLOAD) return false;
  if (n != (size_t)BL_HDR_LEN + h->len + BL_CRC_LEN) return false;
  uint16_t want = (uint16_t)in[BL_HDR_LEN + h->len]
                | (uint16_t)in[BL_HDR_LEN + h->len + 1] << 8;
  if (bl_crc16(in, BL_HDR_LEN + h->len) != want) return false;
  *hdr = *h;
  *payload = in + BL_HDR_LEN;
  return true;
}

// ---------------------------------------------------------------------------
// Empacotamento de records dentro de um payload DATA.
// Cada record é precedido por um byte de tamanho; records maiores que
// BL_REC_MAX_INLINE vão por fragmentação (flags BL_F_FRAG / BL_F_MORE).
// ---------------------------------------------------------------------------
#define BL_REC_MAX_INLINE  (BL_MAX_PAYLOAD - 1)   // 239

static const uint8_t BL_BCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

#endif // BL_PROTO_H
