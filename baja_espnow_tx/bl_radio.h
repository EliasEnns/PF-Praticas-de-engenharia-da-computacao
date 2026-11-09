// ============================================================================
//  BajaLink — camada de rádio: ESP-NOW, long range e compatibilidade de core
// ----------------------------------------------------------------------------
//  ARQUIVO COMPARTILHADO. Mantenha byte-a-byte idêntico em:
//      baja_espnow_tx/bl_radio.h
//      baja_espnow_rx/bl_radio.h
//
//  Duas coisas moram aqui:
//
//  1) Ganho de alcance. Três medidas somadas, da mais barata para a mais
//     agressiva: potência de TX no teto regulatório, desligamento do power
//     save (que de outro modo faria o rádio dormir entre quadros e perder
//     recepções), e o modo 802.11 LR proprietário da Espressif, que troca
//     throughput por sensibilidade. LR a 500 kbps rende cerca de 7 dB sobre
//     802.11b a 1 Mbps; a 250 kbps, cerca de 10 dB. Cada 6 dB equivale a
//     dobrar a distância em espaço livre.
//
//  2) Compatibilidade de assinatura de callback. O core Arduino-ESP32 mudou
//     a assinatura dos callbacks do ESP-NOW duas vezes: o callback de
//     recepção passou a receber esp_now_recv_info_t no IDF 5.0, e o de envio
//     passou a receber wifi_tx_info_t no IDF 5.4. Os macros abaixo deixam os
//     sketches compilarem em qualquer core 2.x ou 3.x sem #ifdef espalhado.
// ============================================================================
#ifndef BL_RADIO_H
#define BL_RADIO_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <stdio.h>

// Os cabecalhos do IDF se reorganizaram entre as geracoes de core: esp_read_mac
// e ESP_MAC_WIFI_STA moraram em esp_system.h e passaram para esp_mac.h, e
// esp_random() seguiu caminho parecido. Inclusao condicional cobre as duas
// geracoes sem travar o sketch numa versao especifica de core.
#if __has_include(<esp_mac.h>)
  #include <esp_mac.h>
#endif
#if __has_include(<esp_random.h>)
  #include <esp_random.h>
#endif
#if __has_include(<esp_system.h>)
  #include <esp_system.h>
#endif

#include "bl_config.h"
#include "bl_proto.h"

// ---------------------------------------------------------------------------
// Compatibilidade: callback de recepção
// ---------------------------------------------------------------------------
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #define BL_RECV_CB(fn) \
      void fn(const esp_now_recv_info_t *bl_info, const uint8_t *data, int len)
  #define BL_RECV_MAC    (bl_info->src_addr)
  #define BL_RECV_RSSI   ((int8_t)(bl_info->rx_ctrl ? bl_info->rx_ctrl->rssi : 0))
#else
  #define BL_RECV_CB(fn) \
      void fn(const uint8_t *bl_src, const uint8_t *data, int len)
  #define BL_RECV_MAC    (bl_src)
  #define BL_RECV_RSSI   ((int8_t)0)
#endif

// ---------------------------------------------------------------------------
// Compatibilidade: callback de envio. Só o status interessa, porque o link
// tem um único peer por vez — o MAC do argumento é deliberadamente ignorado,
// o que evita depender dos nomes de campo de wifi_tx_info_t.
// ---------------------------------------------------------------------------
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  #define BL_SEND_CB(fn) \
      void fn(const wifi_tx_info_t *, esp_now_send_status_t status)
#else
  #define BL_SEND_CB(fn) \
      void fn(const uint8_t *, esp_now_send_status_t status)
#endif

// ---------------------------------------------------------------------------
// Utilidades
// ---------------------------------------------------------------------------
static inline const char *bl_mac_str(const uint8_t *m) {
  static char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           m[0], m[1], m[2], m[3], m[4], m[5]);
  return buf;
}

// Identificador de boot: muda a cada inicializacao, e e assim que cada lado
// detecta que o outro reiniciou. Usa WiFi.macAddress(), da API do Arduino, em
// vez de esp_read_mac() do IDF: a funcao do Arduino existe em qualquer versao
// de core e nao depende de qual cabecalho do IDF declara o que.
static inline uint32_t bl_boot_id(void) {
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  uint32_t id = ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
  return (id << 8) ^ (uint32_t)esp_random();
}

// Nome da taxa em uso, para o log de boot e para o cabeçalho do relatório.
static inline const char *bl_rate_name(void) {
#if BL_LONG_RANGE
  return BL_LR_RATE_500K ? "LR 500 kbps" : "LR 250 kbps";
#else
  return "802.11b/g/n";
#endif
}

// ---------------------------------------------------------------------------
// Troca de canal
//
// Alternar promíscuo em volta da troca é necessário porque, numa STA não
// associada, algumas versões do IDF recusam esp_wifi_set_channel() em
// silêncio. Com o promíscuo ligado a troca sempre vale.
// ---------------------------------------------------------------------------
static inline void bl_radio_set_channel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

static inline uint8_t bl_radio_get_channel(void) {
  uint8_t ch = 0;
  wifi_second_chan_t sec;
  if (esp_wifi_get_channel(&ch, &sec) != ESP_OK) return 0;
  return ch;
}

// ---------------------------------------------------------------------------
// Inicialização do rádio
// ---------------------------------------------------------------------------
static inline void bl_radio_begin(uint8_t channel) {
  WiFi.mode(WIFI_STA);
  // Garante que não há associação a AP: uma STA associada fica presa ao canal
  // do AP e ignora bl_radio_set_channel().
  WiFi.disconnect(false, true);

  // Sem power save o rádio fica com o receptor sempre aberto. Custa corrente,
  // mas sem isso o ESP-NOW perde quadros assincronamente.
  esp_wifi_set_ps(WIFI_PS_NONE);

#if BL_LONG_RANGE == 2
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#elif BL_LONG_RANGE == 1
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                     WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
#endif

  // Teto de potência. A faixa aceita é 8..84 em quartos de dBm; o rádio
  // satura no limite do módulo, então pedir 20 dBm é seguro.
  esp_wifi_set_max_tx_power(BL_TX_POWER_QDBM);

  bl_radio_set_channel(channel);
}

// ---------------------------------------------------------------------------
// Força a taxa PHY do peer.
//
// Sem isso o rate control do Wi-Fi escolhe a taxa sozinho e sobe para
// modulações frágeis assim que o link parece bom, o que num carro em
// movimento significa oscilar entre 54 Mbps e queda total. Fixar a taxa
// baixa é o que transforma o alcance teórico do LR em alcance real.
// Devolve true se alguma taxa foi fixada.
// ---------------------------------------------------------------------------
static inline bool bl_peer_force_rate(const uint8_t *mac) {
#if BL_LONG_RANGE
  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
    esp_now_rate_config_t rc = {};
    rc.phymode = WIFI_PHY_MODE_LR;
    rc.rate = BL_LR_RATE_500K ? WIFI_PHY_RATE_LORA_500K
                              : WIFI_PHY_RATE_LORA_250K;
    if (esp_now_set_peer_rate_config(mac, &rc) == ESP_OK) return true;

    // Degradação graciosa: se o chip não aceita LR, fixa 802.11b a 1 Mbps,
    // que ainda é a modulação mais robusta do conjunto legado.
    rc = esp_now_rate_config_t{};
    rc.phymode = WIFI_PHY_MODE_11B;
    rc.rate    = WIFI_PHY_RATE_1M_L;
    return esp_now_set_peer_rate_config(mac, &rc) == ESP_OK;
  #else
    (void)mac;
    return esp_wifi_config_espnow_rate(
             WIFI_IF_STA, BL_LR_RATE_500K ? WIFI_PHY_RATE_LORA_500K
                                          : WIFI_PHY_RATE_LORA_250K) == ESP_OK;
  #endif
#else
  (void)mac;
  return false;
#endif
}

// ---------------------------------------------------------------------------
// Cadastro de peer. channel = 0 significa "siga o canal atual da interface",
// que é o que permite a varredura de canais funcionar sem recadastrar peers.
// ---------------------------------------------------------------------------
static inline esp_err_t bl_peer_add(const uint8_t *mac, bool encrypt) {
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.ifidx   = WIFI_IF_STA;
  p.encrypt = encrypt;
#if BL_ENCRYPT
  if (encrypt) memcpy(p.lmk, BL_LMK, 16);
#endif
  if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
  esp_err_t e = esp_now_add_peer(&p);
  if (e == ESP_OK) bl_peer_force_rate(mac);
  return e;
}

static inline esp_err_t bl_espnow_begin(void) {
  esp_err_t e = esp_now_init();
  if (e != ESP_OK) return e;
#if BL_ENCRYPT
  esp_now_set_pmk((const uint8_t *)BL_PMK);
#endif
  // O peer de broadcast é sempre em claro: ESP-NOW não cifra broadcast, e a
  // descoberta precisa dele.
  bl_peer_add(BL_BCAST, false);
  return ESP_OK;
}

#endif // BL_RADIO_H
