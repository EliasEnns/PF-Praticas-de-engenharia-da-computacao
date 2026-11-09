# Plano de testes — link BajaLink / ESP-NOW

Ensaios para caracterizar o link. Os três primeiros já foram executados e
estão com os resultados preenchidos; os demais precisam dos dois devkits e das
tabelas preenchidas à mão.

Regra para o relatório final: **nenhum número vem de estimativa.** Todo valor
citado sai de um log no disco ou da saída de um dos testes. Onde não houver
medida, a tabela fica em branco e o texto diz que não foi medido — o que é um
resultado honesto, e muito melhor que um número inventado que não sobrevive a
uma pergunta na apresentação.

---

## T0 — Linha de base do caminho LoRa  ✅ executado

**Objetivo.** Quantificar o que o sistema atual entrega, para que o ganho do
caminho novo seja uma comparação e não uma afirmação.

**Método.** Análise de log real de pista já existente, sem repetir ensaio:

```
python tools/link_report.py "Dash teste Claude/logs/telemetria_20260530_114018.csv"
```

**Resultado.**

| Métrica | Valor medido |
|---|---|
| Arquivo | `telemetria_20260530_114018.csv` |
| Data da sessão | 2026-05-30 11:47 |
| Records no log | 1283 |
| Duração | 715,0 s |
| **Taxa média de entrega** | **1,79 records/s** |
| Intervalo mediano entre records | 550 ms |
| Intervalo p95 | 551 ms |
| Intervalo máximo | 2199 ms |
| Jitter (desvio padrão) | 86,5 ms |
| Buracos no stream | 2 (4398 ms no total) |
| Perda por lacuna em `seq` | 0 |

**Leitura.** O caminho LoRa é confiável — zero perda de sequência em 1283
records — e lento. Com intervalo mediano de 550 ms, o critério de Nyquist
limita a análise a conteúdo de frequência abaixo de **0,9 Hz**. É suficiente
para temperatura, tensão e nível de combustível, que é exatamente o que se
acompanhava. Não serve para transitório de frenagem, resposta de suspensão nem
conteúdo vibratório de estrutura, que são os alvos declarados na proposta.

---

## T1 — Teste de integração do protocolo no PC  ✅ executado

**Objetivo.** Verificar as propriedades do transporte sem depender de hardware
nem de um canal de rádio que não se pode controlar.

**Método.** `tools/test_bajalink.cpp` compila as estruturas reais do firmware
no PC e as exercita contra um canal sintético com perda, atraso e reordenação.

```
g++ -std=c++17 -O1 -I tools/stub -I baja_espnow_tx -I baja_espnow_rx \
    tools/test_bajalink.cpp -o /tmp/test_bajalink && /tmp/test_bajalink
```

**Resultado.** 3000 records por cenário, a 50 records/s.

| Cenário | Entregues | Retx | Lacunas | Estouro | Veredito |
|---|---|---|---|---|---|
| Canal limpo | 3000 / 3000 | 0 | 0 | 0 | integral, sem retx espúria |
| 15 % de perda | 2998 / 3000 | 1127 | 1 | 0 | 99,93 % |
| 30 % de perda | 2931 / 3000 | 2023 | 68 | 267 | 97,7 %, degradação graciosa |
| Apagão de 2 s | 2997 / 3000 | 369 | 3 | **0** | buffer absorveu integralmente |
| Apagão de 30 s | 1752 / 3000 | 223 | 1248 | 1250 | perda esperada e contabilizada |

Verificado em todos os cenários: ordem de entrega preservada, zero duplicatas
entregues, e toda perda refletida nos contadores.

O apagão de 30 s excede a autonomia do buffer sem PSRAM (256 quadros, ~5 s). A
perda de 1248 records corresponde aos 30 s de apagão menos os ~5 s que o
buffer cobriu — o resultado está correto, não é defeito.

Além dos cenários, o teste cobre o formato de fio (CRC rejeita bit invertido,
versão divergente recusada, payload grande demais recusado), a aritmética de
sequência no wrap de 16 bits, a janela de reordenação (fora de ordem,
preenchimento de lacuna, duplicata, encerramento antecipado) e o buffer
(enchimento exato, contabilização de estouro).

**Os defeitos que este ensaio encontrou estão registrados na seção 9 de
`BAJALINK_PROTOCOLO.md`.** Foram cinco, mais um sexto que só apareceu depois de
corrigir os outros cinco.

---

## T2 — Contrato entre firmware e dashboard  ✅ executado

**Objetivo.** Garantir que a linha escrita pelo gateway é exatamente a que o
dashboard lê. É o único ponto do sistema onde C e Python precisam concordar
byte a byte, e o sintoma de divergência é ruim: o dashboard descarta a linha em
silêncio, por tolerância a quadro truncado no boot, e a tela fica parada sem
mensagem de erro.

**Método.** O teste T1 gera 600 linhas com a função **real** do firmware
(`bl_json_decorate`), variando os campos do link para cobrir negativos, zeros e
valores grandes. Em seguida:

```
python tools/test_formato.py
```

que passa essas linhas pelo parser **real** do dashboard.

**Resultado.** Todas as verificações passaram:

- toda linha é JSON válido e traz os objetos `tele` e `link`
- nenhuma chave que o dashboard espera falta no fio, e nenhuma chave que o
  firmware manda é ignorada pelo dashboard (as duas direções)
- os 20 campos do link chegam achatados com o valor exato
- o CSV ganha as 20 colunas e nenhuma fica vazia
- record do caminho LoRa antigo, sem objeto `link`, continua sendo processado
- lixo na porta (JSON truncado, tipos errados, linha vazia) não derruba o parser

| Métrica | Valor medido |
|---|---|
| Linha de saída, pior caso | 457 B |
| Carga a 50 records/s | 22,9 kB/s = 228 kbps |
| Folga sobre o console a 921600 baud | 75 % |

**Leitura.** 228 kbps não caberiam nos 115200 baud do caminho LoRa. Foi por
isso que o console subiu para 921600, e não por precaução.

---

## T3 — Bancada: associação e taxa sustentada  ⬜ a executar

**Objetivo.** Confirmar que o link sobe sozinho e sustenta a taxa nominal com
os dois devkits na mesma mesa, que é a condição de melhor caso.

**Método.**

1. Grave `baja_espnow_rx` num devkit, `baja_espnow_tx` no outro.
2. Monitor serial do gateway a 921600. Anote o MAC e espere a linha
   `# sensor XX:XX:...`.
3. Monitor serial do sensor. `rate 50`, depois `zero` para limpar contadores.
4. Deixe rodar 10 minutos.
5. `stats` nos dois lados.
6. Feche os monitores, abra o dashboard, grave um log de 5 minutos e gere o
   relatório.

```
python telemetria_baja.py
# ... grave o log pela interface ...
python tools/link_report.py "Dash teste Claude/logs/<log>.csv" \
       --baseline "Dash teste Claude/logs/telemetria_20260530_114018.csv" \
       -o docs/RESULTADOS.md
```

| Métrica | Valor medido |
|---|---|
| Tempo até associar, do boot | |
| Taxa nominal / taxa entregue | 50 Hz / |
| Intervalo mediano | |
| Jitter (desvio padrão) | |
| PDR da sessão | |
| Retransmissões (% dos enviados) | |
| Lacunas declaradas | |
| RSSI médio | |
| RTT estimado médio | |
| Janela média | |
| Ocupação de pico do buffer | |
| Ganho sobre a linha de base T0 | ×|

---

## T4 — Varredura de taxa: onde está o joelho  ⬜ a executar

**Objetivo.** Achar a taxa máxima sustentável e mostrar que a escolha de 50 Hz
tem margem, em vez de ser um número arbitrário.

**Método.** Com os dois lados na mesma mesa, para cada taxa: `rate <N>`,
`zero`, espere 2 minutos, `stats` nos dois lados.

| `rate` | Entregue (Hz) | PDR | Retx % | Buffer de pico | Estouro | Observação |
|---|---|---|---|---|---|---|
| 25 | | | | | | |
| 50 | | | | | | padrão |
| 75 | | | | | | |
| 100 | | | | | | |
| 150 | | | | | | |
| 200 | | | | | | |

**O que procurar.** O joelho aparece quando a ocupação do buffer deixa de
voltar a zero entre amostras: isso significa que a produção passou a drenagem.
A 500 kbps em LR um quadro de 250 B leva cerca de 4 ms de ar, o que põe o teto
teórico perto de 200 quadros/s antes de contar overhead e ACK. O valor medido
deve ficar abaixo disso.

---

## T5 — Queda de link e store-and-forward  ⬜ a executar

**Objetivo.** Demonstrar a propriedade central do projeto: queda de link vira
latência, não lacuna, enquanto couber no buffer.

**Método.** Com o dashboard gravando e o log aberto, provoque a queda. Em ordem
de preferência:

1. Leve o sensor para trás de uma parede de concreto ou de um elevador até o
   RSSI cair e o dashboard mostrar `ESP-NOW procurando`.
2. Ou envolva o sensor em papel-alumínio. Funciona, atenua muito.
3. Ou, como último recurso, `ch 11` no sensor com o gateway no canal 1. Testa a
   mecânica do buffer, mas não o comportamento de RF.

Faça três durações e, para cada uma, volte o link e deixe drenar.

| Duração da queda | Buffer de pico | Estouro | Lacunas | Tempo de drenagem | Records perdidos |
|---|---|---|---|---|---|
| ~2 s | | | | | |
| ~10 s | | | | | |
| ~60 s | | | | | |

**O que procurar no painel do dashboard.** Durante a queda, a curva pontilhada
do buffer sobe enquanto o PDR despenca. Na volta, o buffer desce **mais rápido
do que subiu** — a janela de congestionamento abre acima do regime permanente
para drenar o atraso. Se o buffer encosta em 100 % e fica, a queda passou da
autonomia e há perda definitiva, que aparece na coluna de estouro.

**Previsão.** Sem PSRAM, 256 quadros a 50 Hz dão ~5 s de autonomia. A queda de
2 s deve ter estouro **zero**; a de 60 s deve perder cerca de 55 s × 50 =
2750 records. Com PSRAM ligada no IDE, a autonomia vai a ~80 s e a queda de
60 s também deve passar sem perda. Vale fazer as duas versões se o devkit tiver
PSRAM: é o resultado mais vistoso do conjunto.

---

## T6 — Alcance  ⬜ a executar

**Objetivo.** Medir o alcance útil, que é o que justifica o esforço de LR e de
taxa fixa.

**Método.** Gateway parado, alimentado, com o dashboard gravando. Caminhe com o
sensor em linha reta, parando a cada 25 m por 30 s. Use os dois lados do
estacionamento ou a reta mais longa disponível. Anote RSSI e PDR do painel.

| Distância | RSSI | PDR | Retx % | Lacunas | Observação |
|---|---|---|---|---|---|
| 0 m (mesa) | | | | | |
| 25 m | | | | | |
| 50 m | | | | | |
| 100 m | | | | | |
| 150 m | | | | | |
| 200 m | | | | | |
| Queda | | | | | distância em que perdeu |

**Referência de leitura.** Acima de −75 dBm o link tem folga. Entre −75 e
−90 funciona, mas é onde a retransmissão começa a aparecer. Abaixo de −90 está
no limite da sensibilidade e a queda é questão de alguns metros.

Registre obstáculos: linha de visada direta e visada obstruída dão resultados
muito diferentes, e o número sem o contexto não significa nada.

---

## T7 — Modo de rádio: LR 500k, LR 250k, legado  ⬜ a executar

**Objetivo.** Quantificar a troca entre alcance e banda, em vez de citá-la.

**Método.** Para cada configuração, recompile os dois sketches e repita uma
distância de T6 em que o PDR já não seja 100 % — é onde a diferença aparece.
Perto do gateway, todas as três configurações entregam tudo e o ensaio não
mostra nada.

| Configuração | `BL_LONG_RANGE` | `BL_LR_RATE_500K` | RSSI | PDR | Taxa máxima |
|---|---|---|---|---|---|
| LR 500 kbps (padrão) | 1 | 1 | | | |
| LR 250 kbps | 1 | 0 | | | |
| 802.11b/g/n | 0 | — | | | |

---

## T8 — Robustez operacional  ⬜ a executar

**Objetivo.** Verificar que o sistema se recupera sozinho das coisas que
acontecem de fato numa pista.

| Ensaio | Esperado | Resultado |
|---|---|---|
| Reset do sensor com gateway ligado | gateway reancora a numeração, dashboard segue | |
| Reset do gateway com sensor ligado | sensor redescobre, buffer drena | |
| Dashboard fechado com gateway ligado | gateway não travar na escrita da porta | |
| Dashboard reaberto | reconecta sozinho em até 2 s | |
| Sensor desligado 5 min e religado | associa sem intervenção | |
| Cabo USB do gateway removido e recolocado | dashboard reconecta sozinho | |

O terceiro ensaio merece atenção: em USB CDC nativo, escrever sem host lendo
**bloqueia**. O firmware chama `Serial.setTxTimeoutMs(0)` exatamente para isso.
Sem essa linha, fechar o dashboard travaria o gateway, que pararia de mandar
ACK, e o link cairia por um motivo que nada tem a ver com rádio.

---

## T9 — Fonte UART da central STM32  ⬜ a executar, depende da central

**Objetivo.** Fechar o caminho real, com o record vindo da USART3 em vez do
emulador.

**Método.**

1. Em `baja_espnow_tx.ino`, troque `BL_SOURCE` para `BL_SOURCE_UART`.
2. Ligue USART3_TX do STM32 no GPIO18 do ESP32 e os terras. O TX do ESP32
   (GPIO17) é opcional, só serve se quiser mandar comando de volta.
3. Confirme que os dois lados estão a 460800 (`BL_UART_BAUD`).
4. Faça o STM32 emitir o mesmo record JSON, uma linha por record, terminada em
   `\n`.

| Métrica | Valor medido |
|---|---|
| Taxa de records da central | |
| Linhas longas descartadas (`stats` do sensor) | |
| Taxa entregue no dashboard | |
| Divergência entre as duas | |

**Ponto de atenção.** A 50 records/s de ~220 B são 88 kbps líquidos, que não
cabem em 115200 com folga de enquadramento. Daí os 460800. Se a central não
puder subir o baud, a taxa máxima cai por limite de UART e não de rádio — o que
precisa ser dito explicitamente no relatório, porque é uma limitação diferente.
