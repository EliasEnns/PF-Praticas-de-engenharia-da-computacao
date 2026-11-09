# Telemetria Automotiva — datalogging de alta frequência via ESP32 e ESP-NOW

Projeto da trilha formativa de Internet das Coisas. Elias Enns e Wesley
Machado.

Expande o sistema de telemetria do veículo Baja SAE substituindo o caminho de
captura por um link ESP-NOW entre dois ESP32-S3. O ganho não é alcance — LoRa
ganha disso com folga — e sim banda.

| | Caminho LoRa (medido) | Caminho ESP-NOW |
|---|---|---|
| Taxa de entrega | **1,79 records/s** | 50 records/s (configurável até ~150) |
| Intervalo mediano | 550 ms | 20 ms |
| Conteúdo resolvível (Nyquist) | ~0,9 Hz | ~25 Hz |

A linha de base de 1,79 records/s não é estimativa: sai de
`logs/telemetria_20260530_114018.csv`, um log de pista real de 715 s, analisado
por `tools/link_report.py`. A 0,9 Hz se acompanha temperatura, tensão e
combustível. Transitório de frenagem, resposta de suspensão e conteúdo
vibratório de estrutura, que são os alvos declarados na proposta, precisam de
mais.

---

## Arquitetura

```
   ┌─────────────────┐
   │  Central STM32  │  aquisição (já existente)
   │   (no carro)    │
   └────────┬────────┘
            │ USART3, 460800, 1 record JSON por linha
            ▼
   ┌─────────────────┐
   │   ESP32-S3 TX   │  buffer store-and-forward em RAM
   │  "sensor"       │  ARQ: sequência, SACK, retransmissão, janela
   └────────┬────────┘
            │ ESP-NOW + BajaLink  (802.11 LR, taxa PHY fixa, 20 dBm)
            ▼
   ┌─────────────────┐
   │   ESP32-S3 RX   │  reordenação, prazo de lacuna
   │  "gateway"      │  injeta o objeto "link" em cada record
   └────────┬────────┘
            │ USB CDC, 921600, 1 record JSON por linha
            ▼
   ┌─────────────────┐
   │   Dashboard     │  Dash/Plotly, calibração, alertas, playback
   └─────────────────┘
```

A decisão de compatibilidade mais importante: o gateway fala **exatamente o
mesmo protocolo de linha** que o receptor LoRa STM32 já falava. Calibração,
filtros EMA, regras de alerta e playback dos CSVs antigos continuam valendo sem
conversão. O que muda é um objeto `"link"` anexado a cada record, com a saúde
do link.

---

## Estrutura

```
Sensor ESP32/
├── baja_espnow_tx/            sketch do sensor (transmissor)
│   ├── baja_espnow_tx.ino
│   ├── bl_config.h            ──┐
│   ├── bl_proto.h               │ compartilhados: manter
│   ├── bl_radio.h             ──┘ idênticos nos dois sketches
│   ├── bl_store.h             buffer store-and-forward
│   └── bl_emu.h               emulador de telemetria
├── baja_espnow_rx/            sketch do gateway (receptor)
│   ├── baja_espnow_rx.ino
│   ├── bl_config.h  bl_proto.h  bl_radio.h
│   ├── bl_reorder.h           janela de reordenação
│   └── bl_json.h              montagem da linha de saída
├── Dash teste Claude/
│   ├── telemetria_baja.py     dashboard, v3.0
│   └── logs/                  CSVs das sessões
├── docs/
│   ├── BAJALINK_PROTOCOLO.md  especificação do protocolo
│   └── PLANO_DE_TESTES.md     ensaios, com resultados e tabelas a preencher
└── tools/
    ├── test_bajalink.cpp      teste de integração do protocolo, roda no PC
    ├── test_formato.py        contrato firmware ↔ dashboard
    ├── link_report.py         relatório de desempenho a partir de um log
    ├── rodar_testes.sh/.bat   roda os dois testes
    └── stub/                  cabeçalho mínimo para compilar no PC
```

Os três headers compartilhados são cópias idênticas nos dois sketches, porque
o Arduino IDE compila cada sketch isoladamente e não há como apontar para uma
pasta comum sem instalar uma biblioteca. **Ao alterar qualquer um deles, copie
nos dois.** Os dois lados conferem a versão do protocolo no handshake e se
recusam a associar se divergirem, em vez de entregar dados mal interpretados.

---

## Começando

### 1. Gravar os dois devkits

Ferramentas, no Arduino IDE, iguais nos dois sketches:

| Opção | Valor |
|---|---|
| Placa | ESP32S3 Dev Module |
| USB CDC On Boot | **Enabled** se usar a porta USB nativa, **Disabled** se usar a porta UART |
| PSRAM | OPI PSRAM, se o devkit tiver |
| Upload Speed | 921600 |

A opção de PSRAM vale atenção: com ela o buffer do sensor vai de 256 para 4096
quadros, ou seja de ~5 s para ~80 s de autonomia de queda de link. O firmware
detecta em tempo de execução e imprime o que alocou no boot.

### 2. Subir o link

Grave o gateway primeiro e deixe o monitor serial aberto a **921600**. Ele
imprime MAC e canal no boot, e anuncia `# sensor XX:XX:...` quando o outro
associar. Nenhum MAC precisa ser configurado: a descoberta é por broadcast.

### 3. Dashboard

```
cd "Dash teste Claude"
pip install -r requirements.txt
python telemetria_baja.py
```

Abre em `http://127.0.0.1:8050`. A porta é encontrada por VID:PID — o gateway
ESP32 tem precedência, e o receptor LoRa STM32 continua na lista, então os dois
funcionam sem trocar configuração.

---

## Console serial

### Sensor

| Comando | Efeito |
|---|---|
| `help` | lista os comandos |
| `stats` | estado completo do link e dos contadores |
| `rate <hz>` | taxa da fonte, 1 a 400, em tempo real |
| `estop <0\|1>` | força o E-STOP no emulador, para testar alertas |
| `ch <1-13>` | troca de canal e reassocia |
| `zero` | zera os contadores |

### Gateway

| Comando | Efeito |
|---|---|
| `help` · `stats` · `zero` | idem |
| `ch <1-13>` | troca de canal |

As linhas de log do gateway começam com `# `, e o dashboard as descarta como
JSON inválido — dá para deixar o monitor aberto junto do dashboard sem
conflito, desde que só um dos dois tenha a porta aberta de cada vez.

`rate` e `stats` são as duas ferramentas de medição: é com elas que se levanta
a varredura de taxa do ensaio T4 em poucos minutos.

---

## Testes

Dois testes rodam sem hardware nenhum.

```
tools\rodar_testes.bat          # Windows
./tools/rodar_testes.sh         # Linux, macOS, WSL
```

**Teste de integração** (`test_bajalink.cpp`). Compila as estruturas reais do
firmware no PC e as exercita contra um canal sintético com perda, atraso e
reordenação. Verifica formato de fio, aritmética de sequência no wrap de 16
bits, janela de reordenação, buffer, e cinco cenários de ponta a ponta:

| Cenário | Entregues | Veredito |
|---|---|---|
| Canal limpo | 3000 / 3000 | integral, sem retransmissão espúria |
| 15 % de perda | 2998 / 3000 | 99,93 % |
| 30 % de perda | 2931 / 3000 | 97,7 %, degradação graciosa |
| Apagão de 2 s | 2997 / 3000 | zero estouro de buffer |
| Apagão de 30 s | 1752 / 3000 | perda esperada e contabilizada |

Em todos: ordem preservada, zero duplicatas entregues, toda perda refletida nos
contadores.

Este teste encontrou cinco defeitos no protocolo, mais um sexto que só apareceu
depois de corrigir os cinco. Estão registrados na seção 9 de
`docs/BAJALINK_PROTOCOLO.md`, com o que cada um causava e como foi corrigido. O
mais instrutivo: a janela de congestionamento era reduzida a cada ACK que
trazia bits seletivos acesos, mas bits acesos indicam "chegou algo fora de
ordem", condição que persiste durante toda a lacuna e não um novo evento de
perda. A janela desabava para o piso e não voltava a subir, e com 15 % de perda
no canal a entrega caía para 85 %. A correção é a época de recuperação do TCP
NewReno.

**Teste de contrato** (`test_formato.py`). O único ponto do sistema em que C e
Python precisam concordar byte a byte é a linha que o gateway escreve na porta.
O teste de integração gera 600 linhas com a função **real** do firmware, e este
script as passa pelo parser **real** do dashboard, conferindo campo por campo
nas duas direções: chave que o dashboard espera e o firmware não manda, e chave
que o firmware manda e o dashboard ignora. Também verifica que um record do
caminho LoRa antigo, sem o objeto `link`, continua sendo processado.

Isso fecha um risco com sintoma ruim de diagnosticar: por tolerância a quadro
truncado durante o boot do MCU, o dashboard descarta linha malformada em
silêncio. Divergência de formato apareceria como tela parada, sem nenhuma
mensagem de erro.

---

## Relatório de resultados

```
python tools/link_report.py "Dash teste Claude/logs/<log>.csv" \
       --baseline "Dash teste Claude/logs/telemetria_20260530_114018.csv" \
       -o docs/RESULTADOS.md
```

Produz um relatório em Markdown com taxa, cadência e jitter, integridade de
sequência contada de forma independente dos contadores do firmware, estatística
de RSSI e PDR, retransmissões, ocupação de pico do buffer, RTT e janela, e a
comparação com a linha de base LoRa, inclusive o ganho em frequência
resolvível.

Lê tanto os logs novos quanto os antigos sem as colunas de link, o que é o que
permite a comparação.

---

## O que está medido e o que não está

Medido, sem hardware:

- a linha de base LoRa, de log de pista real
- as propriedades do transporte, pelo teste de integração
- o contrato de formato entre firmware e dashboard
- a carga na porta serial: 457 B por linha no pior caso, 228 kbps a 50
  records/s, que é o motivo do console a 921600 e não 115200

Falta medir, precisa dos devkits: ensaios T3 a T9 do plano de testes —
associação e taxa sustentada em bancada, varredura de taxa para achar o joelho,
queda de link e store-and-forward, alcance, comparação entre os modos de
rádio, robustez operacional, e a fonte UART vinda da central.

As tabelas desses ensaios estão em branco de propósito. Número não medido não
sobrevive a uma pergunta na apresentação.

---

## Entrega e escopo

A proposta prevê protótipo funcional como prova de conceito para uma futura
reformulação da placa contemplando o ESP32. É o que está aqui: a placa
revisada (Rev 2.0 da Central de Telemetria Baja) não foi fabricada, e o
adaptador de cartão SD previsto não entrou. Em consequência, o buffer
store-and-forward é volátil — cobre queda de link, não queda de alimentação.
A limitação está registrada na seção 12 de `docs/BAJALINK_PROTOCOLO.md`, junto
das outras.

O emulador de telemetria existe por causa disso, e não é um atalho: gera o
mesmo record, nas mesmas unidades brutas, com estatística de volta realista, o
que permite exercitar e medir o link sem o carro e sem a placa. Trocar para a
fonte real é uma linha em `baja_espnow_tx.ino`:

```c
#define BL_SOURCE  BL_SOURCE_UART
```
