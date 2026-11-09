// ============================================================================
//  BajaLink — configuração compartilhada
//  Telemetria Baja SAE / trilha IoT — Elias Enns, Wesley Machado
// ----------------------------------------------------------------------------
//  ARQUIVO COMPARTILHADO. Mantenha byte-a-byte idêntico em:
//      baja_espnow_tx/bl_config.h
//      baja_espnow_rx/bl_config.h
//  Os dois sketches negociam a versão do protocolo no HELLO; se divergirem,
//  o link se recusa a associar em vez de entregar dados corrompidos.
// ============================================================================
#ifndef BL_CONFIG_H
#define BL_CONFIG_H

// ---------------------------------------------------------------------------
// Versão do protocolo. Incremente ao mudar qualquer estrutura de bl_proto.h.
// ---------------------------------------------------------------------------
#define BL_PROTO_VER            3

// ---------------------------------------------------------------------------
// Rádio
// ---------------------------------------------------------------------------
// Canal "casa". O gateway (RX) fica fixo nele; o sensor (TX) é quem varre a
// lista de canais quando perde o link, garantindo rendezvous determinístico.
#define BL_CHANNEL_HOME         1
#define BL_CHANNEL_LIST         { 1, 6, 11 }
#define BL_CHANNEL_LIST_LEN     3

// Potência de TX em quartos de dBm (80 = 20 dBm). Faixa válida: 8..84.
#define BL_TX_POWER_QDBM        80

// Long Range: modo proprietário 802.11 LR da Espressif. Ganha cerca de 10 dB
// de orçamento de link contra 802.11b 1 Mbps, ao custo de throughput.
//   0 = desligado (802.11b/g/n, maior banda, menor alcance)
//   1 = LR habilitado junto do legado, taxa forçada por peer  <-- recomendado
//   2 = LR exclusivo (máximo alcance, invisível a Wi-Fi comum)
#define BL_LONG_RANGE           1

// Taxa LR: 0 = 250 kbps (mais alcance), 1 = 500 kbps (mais banda).
// Em 250 kbps um frame de 250 B ocupa ~8 ms de ar; acima de ~90 frames/s o
// link satura. Veja docs/PLANO_DE_TESTES.md, ensaio T4.
#define BL_LR_RATE_500K         1

// ---------------------------------------------------------------------------
// Enquadramento
// ---------------------------------------------------------------------------
// ESP-NOW v1 entrega no máximo 250 B por frame. 250 - 8 (cabeçalho) - 2 (CRC)
// = 240 B de payload útil.
#define BL_MAX_PAYLOAD          240
#define BL_MAX_RECORD           512   // record maior que isso é fragmentado
#define BL_AGG_MAX_RECORDS      4     // records empacotados por frame
#define BL_AGG_MAX_LATENCY_MS   20    // teto de latência do empacotamento

// ---------------------------------------------------------------------------
// ARQ (retransmissão seletiva com janela deslizante)
// ---------------------------------------------------------------------------
// Piso da janela. Nao e 2 por um motivo de projeto: este link tem um unico
// fluxo ponto a ponto, e a janela existe para nao estourar a fila do radio,
// nao para ser justa com vizinhos. Um piso baixo demais faz o AIMD assentar
// abaixo da taxa de aquisicao sob perda aleatoria alta, e entao o buffer
// estoura por controle de congestionamento, nao por falta de radio. Com 4
// quadros e RTT tipico de 25 ms, o piso sustenta ~160 quadros/s.
#define BL_CWND_MIN             4
#define BL_CWND_INIT            8
#define BL_CWND_MAX             32
#define BL_HW_QUEUE             4     // esp_now_send() em voo simultâneos
#define BL_RTO_MIN_MS           25
#define BL_RTO_MAX_MS           400
// Folga fixa somada ao RTO estimado. Cobre a granularidade do relogio e o
// atraso do agrupamento de ACK no gateway. Ver o comentario em
// bl_rtt_update(), no sketch do sensor: sem margem, o RTO converge para o
// proprio RTT num link estavel e passa a disparar retransmissao espuria.
#define BL_RTO_SLACK_MS         10
// Tentativas antes de desistir do quadro. O numero esta acoplado ao prazo de
// lacuna do gateway (BL_RX_HOLE_TIMEOUT_MS): com RTO tipico de ~50 ms e recuo
// exponencial, caberam 4 ou 5 tentativas em 600 ms. Insistir alem disso gasta
// ar com um quadro que o gateway ja declarou perdido e seguiu adiante, o que
// nao recupera nada e atrasa a drenagem do resto. Note que a tentativa so e
// contada quando o quadro e de fato transmitido: durante uma queda de link
// nada e gasto, e o store-and-forward continua guardando normalmente.
#define BL_MAX_TRIES            6
#define BL_ACK_EVERY_N          4     // ACK a cada N frames em regime normal
#define BL_ACK_MAX_DELAY_MS     15
#define BL_REQACK_EVERY         16    // frames entre pedidos de ACK imediato

// ---------------------------------------------------------------------------
// Buffer store-and-forward (RAM)
// ---------------------------------------------------------------------------
// Sem PSRAM usa SRAM interna; com PSRAM detectada em runtime aloca a versão
// profunda. Cada slot custa ~256 B.
#define BL_SLOTS_INTERNAL       256   // ~5 s a 50 frames/s
#define BL_SLOTS_PSRAM          4096  // ~80 s a 50 frames/s
// Política de estouro: 1 = descarta o mais antigo (mantém o dado recente,
// melhor para dashboard ao vivo), 0 = descarta o novo (mantém continuidade).
#define BL_OVERFLOW_DROP_OLDEST 1

// ---------------------------------------------------------------------------
// Reordenação no RX
// ---------------------------------------------------------------------------
// Dimensionamento acoplado, e a primeira versao errou nisso: a janela tem de
// cobrir TODO o periodo em que o gateway fica esperando um quadro faltante,
// porque nesse intervalo os quadros seguintes continuam chegando e precisam
// de lugar. A regra e
//
//     BL_RX_WIN >= BL_RX_HOLE_TIMEOUT_MS * taxa_maxima / 1000 + BL_CWND_MAX
//
// Com 600 ms de prazo e 200 quadros/s: 120 + 32 = 152. Com 100 quadros/s, que
// e o teto pratico deste link, 60 + 32 = 92. Com janela menor que isso, a
// janela estoura antes do prazo expirar e o caminho de excecao passa a ser o
// caminho normal, o que destroi a taxa de entrega.
#define BL_RX_WIN               128
// Tempo que o gateway espera por um quadro faltante antes de declarar lacuna
// e seguir entregando. Limita a latencia de tela no pior caso; 600 ms da
// espaco para duas ou tres retransmissoes com o RTO tipico deste link.
#define BL_RX_HOLE_TIMEOUT_MS   600

// ---------------------------------------------------------------------------
// Descoberta, keepalive e telemetria do próprio link
// ---------------------------------------------------------------------------
#define BL_HELLO_PERIOD_MS      300
#define BL_LINK_LOST_MS         1500  // silêncio que caracteriza link caído
#define BL_CH_DWELL_MS          600   // permanência por canal na varredura
// O STAT nao e so telemetria: ele carrega tx_base_seq, que e o que permite ao
// gateway encerrar lacunas definitivas sem gastar o prazo inteiro. Por isso
// 250 ms e nao 500: o periodo do STAT e o teto da latencia de recuperacao
// depois de um estouro de buffer no sensor.
#define BL_STAT_PERIOD_MS       250   // período do frame STAT (TX -> RX)
#define BL_LINK_LINE_MS         250   // período da linha $BL no console do RX

// ---------------------------------------------------------------------------
// Portas seriais
// ---------------------------------------------------------------------------
// Console / saída de dados. 921600 porque 50 records/s de ~210 B exigem
// ~105 kbps líquidos, o que não cabe em 115200.
#define BL_CONSOLE_BAUD         921600

// UART de entrada no sensor (USART3 livre do STM32 -> UART1 do ESP32).
// Só usada quando BL_SOURCE == BL_SOURCE_UART.
#define BL_UART_BAUD            460800
#define BL_UART_RX_PIN          18    // ESP32 RX  <- STM32 USART3 TX
#define BL_UART_TX_PIN          17    // ESP32 TX  -> STM32 USART3 RX (opcional)

// ---------------------------------------------------------------------------
// Criptografia ESP-NOW
// ---------------------------------------------------------------------------
// Desligada por padrão: a descoberta por broadcast não funciona cifrada e o
// payload é telemetria de um protótipo de pista. Se ligar, fixe o MAC do
// gateway em BL_PEER_MAC_FIXED e pule a descoberta.
#define BL_ENCRYPT              0
#define BL_PMK                  "BajaSAE_UniSATC_"  // exatamente 16 bytes
#define BL_LMK                  "BajaLink_TelemV3"  // exatamente 16 bytes

#endif // BL_CONFIG_H
