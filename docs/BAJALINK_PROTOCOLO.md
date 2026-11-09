# BajaLink — especificação do protocolo

Camada de transporte confiável sobre ESP-NOW, para o sistema de telemetria do
Baja SAE.

Versão do protocolo: **3** (campo `ver` do cabeçalho). Os dois lados conferem a
versão no HELLO e se recusam a associar se divergirem, em vez de entregar dados
mal interpretados.

---

## 1. Por que uma camada em cima do ESP-NOW

O ESP-NOW entrega quadro íntegro ou não entrega nada: ele tem FCS do 802.11,
confirmação de nível MAC e, opcionalmente, CCMP. O que ele não tem é tudo que
um datalogger precisa.

| Precisa | ESP-NOW dá? |
|---|---|
| Integridade do quadro | sim |
| Ordenação entre quadros | não |
| Detecção de perda | não |
| Retransmissão em nível de aplicação | não |
| Armazenamento do que não foi aceito | não |
| Visibilidade da saúde do link | só RSSI do último quadro |

A confirmação de nível MAC do ESP-NOW parece resolver a retransmissão, mas não
resolve: ela diz que o quadro chegou ao rádio do outro lado, não que chegou ao
destino útil. O quadro pode ser confirmado no ar e descartado em seguida por
fila cheia no receptor. Para um log que vai embasar análise de dinâmica, essa
diferença importa.

O BajaLink acrescenta sequência, ACK cumulativo com extensão seletiva,
retransmissão por lacuna confirmada e por temporizador, janela de
congestionamento adaptativa, buffer store-and-forward em RAM, e um canal de
telemetria do próprio link.

---

## 2. Formato de quadro

Limite do ESP-NOW v1: 250 B por quadro. O cabeçalho consome 8 B e o CRC 2 B,
sobrando 240 B de payload.

```
 0        1        2        3        4        5        6        7
+--------+--------+--------+--------+--------+--------+--------+--------+
| magic  |  ver   |  type  | flags  |      seq (LE)   |    len (LE)     |
|  0xB7  |   3    |        |        |                 |                 |
+--------+--------+--------+--------+--------+--------+--------+--------+
|                        payload (0..240 B)                             |
+-----------------------------------------------------------------------+
|                      CRC16-CCITT (2 B, LE)                            |
+-----------------------------------------------------------------------+
```

O CRC é redundante em relação ao FCS do 802.11, e isso é deliberado: ele cobre
o caminho inteiro, incluindo o salto UART entre a central STM32 e o ESP32, onde
um byte perdido por overrun não seria detectado por mais nada. Polinômio
0x1021, inicialização 0xFFFF, implementação bit a bit sem tabela — a 240 B por
quadro e 50 quadros/s custa menos de 0,1 % de um núcleo a 240 MHz e economiza
512 B de RAM.

### Tipos

| Valor | Tipo | Sentido | Payload |
|---|---|---|---|
| 0x01 | `DATA` | sensor → gateway | records ou fragmento |
| 0x02 | `ACK` | gateway → sensor | `bl_ack_t` |
| 0x03 | `STAT` | sensor → gateway | `bl_stat_t` |
| 0x04 | `HELLO` | sensor → broadcast | `bl_hello_t` |
| 0x05 | `HELLO_ACK` | gateway → sensor | `bl_hello_t` |

### Flags

| Bit | Nome | Significado |
|---|---|---|
| 0x01 | `MORE` | há continuação deste record no próximo quadro |
| 0x02 | `FRAG` | payload é fragmento de um único record |
| 0x04 | `RETX` | é retransmissão; não conta como quadro novo no PDR |
| 0x08 | `GAP` | houve descarte por estouro de buffer antes deste quadro |
| 0x10 | `REQACK` | peço ACK imediato, não espere o agrupamento |

### Payload de `DATA`

Sem `FRAG`: agregado de records, cada um precedido por um byte de tamanho.

```
[len][record ...][len][record ...]...
```

Com `FRAG`: pedaço cru de um record maior que 239 B, com `MORE` aceso em todos
menos o último.

A agregação existe para amortizar o custo fixo de quadro quando os records são
pequenos. Com o record atual de telemetria em cerca de 220 B, cabe um por
quadro na prática — a eficiência de payload fica em 220/250, ou 88 %, e o
mecanismo só passa a render se o record encolher. O ESP-NOW v2, disponível a
partir do IDF 5.4, levanta o limite de quadro para 1490 B e permitiria agregar
seis records; não foi usado para não exigir uma versão mínima de core Arduino.

---

## 3. ACK cumulativo com extensão seletiva

```c
typedef struct __attribute__((packed)) {
  uint16_t ack_base;    // próximo seq esperado; tudo abaixo foi entregue
  uint32_t sack_bits;   // bit i = ack_base + 1 + i chegou fora de ordem
  uint16_t echo_seq;    // seq do DATA que provocou este ACK (medida de RTT)
  uint32_t rx_frames, rx_dups, rx_gaps;
  int8_t   rssi;        // do último DATA, visto pelo gateway
  uint8_t  rx_free;     // espaço livre na janela de reordenação
} bl_ack_t;
```

`ack_base` libera o buffer do sensor em bloco. `sack_bits` descreve as 32
sequências seguintes: um bit apagado com bits mais altos acesos é uma **lacuna
confirmada**, e dispara retransmissão imediata sem esperar o temporizador.

`rx_free` é contrapressão: diz ao sensor quanto espaço resta na janela de
reordenação do gateway.

### Política de envio de ACK

| Condição | Quando |
|---|---|
| Imediato | quadro com `REQACK`, duplicata recebida, lacuna detectada, janela cheia |
| Agrupado | a cada 4 quadros, ou 15 ms depois do primeiro quadro não confirmado |

O agrupamento reduz tráfego de volta; o ACK imediato nas condições acima é o
que mantém a latência de recuperação baixa onde ela importa. O ACK por lacuna é
limitado a um a cada 30 ms, para não virar enxurrada.

---

## 4. Retransmissão

Dois gatilhos.

**Lacuna confirmada.** Chegou um ACK com bits seletivos acesos acima da base,
logo a base é uma lacuna real. Retransmite na hora. Limitado a uma
retransmissão rápida por RTT para a mesma lacuna.

**Temporizador.** RTO estimado conforme RFC 6298, com recuo exponencial por
quadro:

```
rttvar = 3/4 · rttvar + 1/4 · |srtt − amostra|
srtt   = 7/8 · srtt + 1/8 · amostra
rto    = max(srtt + 4·rttvar, 1,25·srtt) + 10 ms,  limitado a [25, 400] ms
```

Amostras de RTT vêm só de quadros transmitidos uma única vez — algoritmo de
Karn. Usar o tempo de uma retransmissão contamina o estimador, porque não se
sabe qual das cópias foi confirmada.

O piso multiplicativo de 1,25·srtt e a folga de 10 ms não estão na forma pura
do RFC, e foram acrescentados por um motivo concreto descrito na seção 9.

Cada quadro tem no máximo 6 tentativas. O número está acoplado ao prazo de
lacuna do gateway: insistir além do prazo gasta ar com um quadro que o outro
lado já declarou perdido. A tentativa só é contada quando o quadro é de fato
transmitido, então uma queda de link não consome tentativas e o
store-and-forward continua guardando normalmente.

---

## 5. Janela de congestionamento

AIMD com época de recuperação, no modelo do TCP NewReno.

| Evento | Efeito |
|---|---|
| ACK que avança a base, fora de recuperação | +1 quadro por RTT |
| Perda detectada, primeira do evento | janela pela metade, piso 4 |
| Perda detectada, mesmo evento | nada |
| Base passa do ponto onde a perda foi detectada | sai da recuperação |
| Queda de link | janela volta ao valor inicial |
| Buffer acima de 50 % | não reduz |

Faixa: 4 a 32 quadros, inicial 8.

Duas escolhas merecem justificativa.

**O piso é 4, não 2.** Este link tem um único fluxo ponto a ponto, e a
concorrência real no ar já está limitada a 4 envios simultâneos pela fila do
rádio. A janela do ARQ dimensiona o pipeline, não o acesso ao meio. Um piso
baixo demais faz o AIMD assentar abaixo da taxa de aquisição sob perda alta, e
aí o buffer estoura por controle de congestionamento, não por falta de rádio.

**Não reduz com buffer cheio.** A fonte de dados é de taxa fixa: ela não
desacelera porque o link piorou. Com backlog acumulado, fechar a janela
atrasa a drenagem e aproxima o estouro, que é perda definitiva. Trocar perda
definitiva por cortesia com um meio que ninguém mais está usando é um mau
negócio.

---

## 6. Store-and-forward

Buffer circular no sensor, indexado por `seq % N`, guardando os quadros ainda
não confirmados.

| | Sem PSRAM | Com PSRAM |
|---|---|---|
| Slots | 256 | 4096 |
| Memória | ~64 KB em SRAM | ~1 MB em PSRAM |
| Autonomia a 50 quadros/s | ~5 s | ~80 s |

A profundidade é escolhida em tempo de execução: o firmware detecta PSRAM e,
se não houver heap suficiente, reduz a profundidade pela metade até caber, para
nunca travar o boot por falta de memória.

Política de estouro: descarta o **mais antigo** e acende `GAP` no quadro
seguinte. Para um dashboard ao vivo, os últimos segundos valem mais que os
primeiros de uma queda longa. O comportamento é invertido em uma linha
(`BL_OVERFLOW_DROP_OLDEST`) se a prioridade passar a ser continuidade.

Duas perdas possíveis, contabilizadas **separadamente** porque têm causas e
soluções diferentes:

| Contador | Causa | O que fazer |
|---|---|---|
| estouro de buffer | a queda durou mais que a autonomia | buffer maior, ou cartão SD |
| abandono por prazo | aquele quadro não foi recuperado no teto de latência | link melhor |

Somados num único número, não se sabe qual dos dois problemas se tem.

O buffer **não é persistente**: um reset do sensor perde o conteúdo. Era o
papel do cartão SD previsto na proposta. Sem a placa, o buffer em RAM cobre
queda de link, não queda de alimentação.

---

## 7. Reordenação e lacunas no gateway

Janela de 128 posições, indexada por `seq % 128`, entrega em sequência estrita.

O dimensionamento é acoplado e errá-lo quebra o link inteiro:

```
BL_RX_WIN ≥ BL_RX_HOLE_TIMEOUT_MS × taxa_máxima / 1000 + BL_CWND_MAX
```

Durante a espera por um quadro faltante, os quadros seguintes continuam
chegando e precisam de lugar. Com 600 ms de prazo e 100 quadros/s: 60 + 32 =
92, dentro dos 128. Com janela menor que isso, a janela estoura antes do prazo
expirar e o caminho de exceção passa a ser o caminho normal.

Três maneiras de encerrar uma lacuna:

1. **O quadro chega**, retransmitido. Caso normal.
2. **Prazo de 600 ms expira** com algo adiante esperando. Declara a lacuna,
   registra e segue. Limita a latência de tela no pior caso.
3. **O sensor informa que desistiu.** O quadro `STAT` carrega `tx_base_seq`:
   nada abaixo dessa base será retransmitido. O gateway encerra na hora todas
   as lacunas mortas, alternando entrega e avanço até não haver mais progresso.
   É a mesma ideia do FORWARD-TSN do SCTP em modo parcialmente confiável.

O item 3 não é refinamento: sem ele, depois de um estouro de buffer no sensor
há dezenas de lacunas definitivas enfileiradas, e resolver uma por prazo faria
a base do gateway andar mais devagar que a produção de dados — o link nunca
se recuperaria.

O prazo só conta quando há algo adiante na janela, isto é, quando se sabe que
um quadro específico está faltando. Sensor que simplesmente parou de
transmitir não gera lacuna.

---

## 8. Alcance, canal e descoberta

### Alcance

Três medidas somadas:

| Medida | Ganho | Custo |
|---|---|---|
| Potência de TX no teto (20 dBm) | — | corrente |
| Power save desligado | evita perda assíncrona de quadro | corrente |
| 802.11 LR a 500 kbps | ~7 dB sobre 11b a 1 Mbps | banda |
| 802.11 LR a 250 kbps | ~10 dB sobre 11b a 1 Mbps | mais banda |

Cada 6 dB equivale a dobrar a distância em espaço livre. O LR é um modo
proprietário da Espressif, disponível no ESP32, S2, S3 e C3; entre dois ESP32
funciona, e com Wi-Fi comum não.

A taxa PHY é **fixada por peer**, e isso é essencial: sem fixar, o controle
automático de taxa sobe para modulações frágeis assim que o link parece bom,
o que num carro em movimento significa oscilar entre 54 Mbps e queda total.
Fixar a taxa baixa é o que transforma o alcance teórico do LR em alcance real.
Se o chip recusar LR, o firmware cai para 802.11b a 1 Mbps e registra isso.

A 250 kbps um quadro de 250 B ocupa cerca de 8 ms de ar, o que satura em torno
de 90 quadros/s. A 500 kbps, cerca de 4 ms. O padrão é 500 kbps; a troca entre
alcance e banda é uma linha em `bl_config.h` e está prevista no ensaio T4 do
plano de testes.

### Descoberta

O sensor emite `HELLO` em broadcast a cada 300 ms; o gateway responde
`HELLO_ACK` unicast e cadastra o peer. Nenhum MAC é fixado em código, o que
evita reconfigurar firmware ao trocar de placa. Há `BL_ENCRYPT` para ligar
CCMP, e nesse caso o MAC precisa ser fixado, porque ESP-NOW não cifra
broadcast.

### Rendezvous de canal

Varredor rápido e varredor lento, para garantir sobreposição:

| | Sensor | Gateway |
|---|---|---|
| Começa a varrer | assim que perde o link | após 10 s de silêncio |
| Permanência por canal | 600 ms | 3000 ms |

Se os dois varressem na mesma cadência, poderiam ficar em fase oposta e nunca
se cruzar. Com a permanência do gateway maior que uma volta completa do sensor,
o encontro é garantido. Lista padrão: canais 1, 6 e 11.

---

## 9. Defeitos encontrados no teste de integração

`tools/test_bajalink.cpp` compila as estruturas reais do firmware no PC e as
exercita contra um canal com perda, atraso e reordenação sintéticos. Encontrou
cinco defeitos, todos em interações que não aparecem lendo o código
isoladamente. O registro fica aqui porque as correções são o que o protocolo
realmente é.

**1. Janela de reordenação menor que o prazo de lacuna.** Janela de 64 com
prazo de 1200 ms a 50 quadros/s: a janela enchia em 1,28 s, antes do prazo
expirar. O caminho de exceção virava o caminho normal. Corrigido com a relação
de dimensionamento da seção 7 — janela 128, prazo 600 ms.

**2. Caminho de janela cheia destrutivo.** Ao receber uma sequência fora de
alcance, o código limpava a janela inteira e reancorava. Jogava fora quadros
corretamente recebidos que ainda seriam entregues, transformando lacuna pequena
em perda grande. Agora só avança sobre lacunas; se a cabeça já chegou, a
entrega drena a janela e o quadro de fora é retransmitido. Nenhum quadro
recebido é descartado para abrir espaço.

**3. Janela reduzida a cada ACK, não a cada perda.** A redução disparava
sempre que um ACK trazia bits seletivos acesos. Mas bits acesos significam
"chegou algo fora de ordem", condição que persiste durante **toda** a lacuna, e
não um novo evento de perda. Com ACK a cada 15 ms, a janela era cortada quase a
cada ACK, desabava para o piso e não voltava a subir. A vazão ficava abaixo da
taxa de aquisição e o buffer vivia estourado: com 15 % de perda no canal, a
entrega caía para 85 %. Corrigido com a época de recuperação da seção 5.

**4. Retransmissão rápida sem limite por RTT.** Cada ACK recebido durante uma
lacuna gerava uma cópia do quadro. O quadro esgotava as tentativas em menos de
200 ms e era abandonado, convertendo uma perda recuperável em lacuna
definitiva. Agora é uma retransmissão rápida por RTT para a mesma lacuna.

**5. "Em voo" definido como "enviado e não confirmado".** Parece natural, mas o
ACK é cumulativo e a extensão seletiva só alcança 32 sequências: com uma lacuna
parada na base, centenas de slots ficavam eternamente nessa condição. A
contagem estourava a janela, nenhum quadro novo saía, e o buffer passava a
descartar por congestionamento inventado. Em voo passou a ser "transmitido e
ainda dentro do seu RTO" — passado o RTO o quadro não está em voo, está
aguardando retransmissão, e não deve ocupar lugar na janela.

**E um sexto, que só apareceu depois de corrigir os outros:** com o canal
perfeito e o estimador de RTT funcionando, `rttvar` convergia para zero e o RTO
para o próprio RTT médio. A partir daí a verificação de expiração rodava no
mesmo milissegundo em que o ACK chegava, e o resultado era quase uma
retransmissão espúria por quadro — num canal **sem perda nenhuma**. É o ponto
cego da forma pura do RFC 6298. Corrigido com o piso multiplicativo e a folga
fixa da seção 4.

---

## 10. Parâmetros

Todos em `bl_config.h`, idêntico nos dois sketches.

| Parâmetro | Padrão | Observação |
|---|---|---|
| `BL_MAX_PAYLOAD` | 240 | limite do ESP-NOW v1 menos cabeçalho e CRC |
| `BL_AGG_MAX_RECORDS` | 4 | records por quadro |
| `BL_AGG_MAX_LATENCY_MS` | 20 | teto de latência do agrupamento |
| `BL_CWND_MIN / INIT / MAX` | 4 / 8 / 32 | janela em quadros |
| `BL_HW_QUEUE` | 4 | envios simultâneos ao rádio |
| `BL_RTO_MIN / MAX / SLACK_MS` | 25 / 400 / 10 | temporizador |
| `BL_MAX_TRIES` | 6 | acoplado ao prazo de lacuna |
| `BL_ACK_EVERY_N` | 4 | agrupamento de ACK |
| `BL_ACK_MAX_DELAY_MS` | 15 | teto do agrupamento |
| `BL_SLOTS_INTERNAL / PSRAM` | 256 / 4096 | profundidade do buffer |
| `BL_RX_WIN` | 128 | ver relação de dimensionamento |
| `BL_RX_HOLE_TIMEOUT_MS` | 600 | teto de latência de tela |
| `BL_LINK_LOST_MS` | 1500 | silêncio que caracteriza queda |
| `BL_STAT_PERIOD_MS` | 250 | teto da recuperação após estouro |
| `BL_LONG_RANGE` | 1 | 0 off, 1 LR + legado, 2 LR exclusivo |
| `BL_LR_RATE_500K` | 1 | 0 = 250 kbps, mais alcance |
| `BL_CHANNEL_HOME` | 1 | canal de casa do gateway |
| `BL_CONSOLE_BAUD` | 921600 | 228 kbps medidos a 50 Hz não cabem em 115200 |
| `BL_UART_BAUD` | 460800 | entrada da USART3 do STM32 |

---

## 11. Saída do gateway

Uma linha, um objeto JSON. O record original intacto, mais dois objetos
anexados. É o mesmo protocolo de linha do receptor LoRa, de propósito:
calibração, filtros, alertas e playback de logs antigos continuam valendo.

`"tele"` é anexado só se o record não trouxer o seu, preservando os campos que
o dashboard já exibia, agora alimentados pelos contadores do ESP-NOW.

`"link"` é sempre anexado. Chaves curtas porque isto vai em todo record: a
50 records/s, cada byte aqui custa 50 B/s na porta serial.

| Chave | Significado | Chave | Significado |
|---|---|---|---|
| `rssi` | RSSI do último DATA, dBm | `buf` | buffer do sensor em uso |
| `pdr` | entrega em janela de 1 s, % | `bmax` | buffer total |
| `fr` | quadros entregues | `cwnd` | janela, quadros |
| `dup` | duplicatas descartadas | `srtt` | RTT estimado, ms |
| `gap` | lacunas declaradas | `rto` | temporizador, ms |
| `ro` | fila de reordenação | `ch` | canal |
| `retx` | retransmissões | `lr` | modo long range |
| `drp` | records perdidos por estouro | `src` | 0 emulador, 1 UART |
| `gu` | quadros abandonados por prazo | `hz` | taxa medida da fonte |
| `up` | 1 se associado | `ld` | quedas de link |

Linha completa medida: **457 B** no pior caso, o que dá 228 kbps a 50
records/s. É por isso que o console roda a 921600 e não a 115200.

O contrato é verificado automaticamente: `tools/test_bajalink.cpp` gera linhas
com a função real do firmware e `tools/test_formato.py` as passa pelo parser
real do dashboard, conferindo campo por campo nas duas direções — chave que o
dashboard espera e o firmware não manda, e chave que o firmware manda e o
dashboard ignora.

---

## 12. Limitações conhecidas

- **Buffer volátil.** Reset do sensor perde o buffer. Cartão SD resolveria;
  não há placa.
- **Um sensor por gateway.** O gateway associa um peer de cada vez. Vários
  sensores exigiriam demultiplexação por MAC e uma janela de reordenação por
  origem.
- **Agregação pouco efetiva.** Com record de 220 B cabe um por quadro. Render
  exigiria encolher o record ou ESP-NOW v2.
- **Sem criptografia por padrão.** `BL_ENCRYPT` existe, mas obriga a fixar o
  MAC, porque broadcast não é cifrado.
- **Entrega não é garantida, é limitada.** O link recupera o que der para
  recuperar dentro do teto de latência e da autonomia do buffer; o resto vira
  contador. Isso é escolha de projeto, não limitação acidental: para
  telemetria ao vivo, tela congelada é pior que lacuna registrada.
