#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dashboard de Telemetria Baja SAE - v3.0
========================================
Caminho de aquisicao atual (trilha IoT):

    Central STM32 --USART3--> ESP32-S3 --ESP-NOW/BajaLink--> ESP32-S3 --USB CDC--> este dashboard

O caminho LoRa original continua suportado sem nenhuma alteracao de
configuracao: o receptor STM32 esta na lista USB_RECEIVERS e a primeira porta
que casar vence. O gateway ESP-NOW fala o MESMO protocolo de linha de antes,
uma linha JSON por record, e apenas acrescenta o objeto "link" com a saude do
link. Calibracao, filtros EMA, alertas e playback de CSVs antigos seguem
valendo sem conversao.

O que a v3 acrescenta sobre a v2:
  - Reconhecimento do gateway ESP32-S3 por VID:PID, com taxa por dispositivo
  - Parsing do objeto "link" e achatamento em link_<campo>, o que faz os
    graficos, tiles e CSV tratarem a saude do link como qualquer outra serie
  - Painel "Link ESP-NOW" com RSSI, PDR, ocupacao do buffer do sensor,
    retransmissoes, janela de congestionamento e RTT estimado
  - Colunas de link no CSV, o que permite analise posterior do link e
    alimenta tools/link_report.py

Features v2:
  - Aquisição serial robusta (reconexão automática, thread-safe, JSON malformado tolerado)
  - Sequence tracking (mod 2^32, wraparound window=100) p/ detectar perdas/resets USB
  - CSV logging com timestamp duplo (PC ISO + MCU ms)
  - Calibração linear persistida (mult + offset) em config_calibracao.json
  - Filtro EMA (Exponential Moving Average) configurável por variável
  - Engine de alertas baseada em regras (>, <, >=, <=, ==) com log persistido
  - Painel de atitude 3D (pitch/roll a partir do acelerômetro, atan2 numericamente estável)
  - Modo playback com Play/Pause/Stop/Velocidade respeitando delta de tempo entre pacotes
  - Atualizações otimizadas via dash.Patch() (extend/del incremental, sem rebuild de figura)
  - Logging estruturado via módulo logging (níveis INFO/WARNING/ERROR)

Estrutura de pastas (auto-criada na primeira execução):
    telemetria_baja.py
    requirements.txt
    logs/                       <- CSVs gravados durante operação live
    config_calibracao.json      <- persistido pela aba Configuração
    config_alertas.json         <- persistido pela aba Alertas
    config_filtros.json         <- persistido pela aba Configuração (alphas EMA)

Execução:
    pip install -r requirements.txt
    python telemetria_baja.py
    -> http://127.0.0.1:8050
"""

from __future__ import annotations

import csv
import json
import logging
import math
import threading
import time
import uuid
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import plotly.graph_objects as go
import serial
import serial.tools.list_ports
from dash import (
    Dash,
    Input,
    Output,
    Patch,
    State,
    ctx,
    dash_table,
    dcc,
    html,
    no_update,
)

# =============================================================================
# SECTION 1 — CONFIG & LOGGING
# =============================================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
# Silencia ruído do werkzeug/dash em produção
logging.getLogger("werkzeug").setLevel(logging.WARNING)
log = logging.getLogger("telemetria")

# Hardware
#
# O gateway deixou de ser exclusivamente o receptor LoRa STM32: no caminho
# novo e um ESP32-S3 rodando o firmware BajaLink. Os dois ficam na lista para
# que o dashboard abra qualquer um dos dois sem precisar de ajuste manual --
# a primeira porta presente que casar vence.
#
#   303A:1001  Espressif USB CDC/JTAG   ESP32-S3, porta USB nativa
#   303A:4001  Espressif TinyUSB CDC    ESP32-S3, porta USB nativa
#   10C4:EA60  CP2102N                  ESP32-S3-DevKitC-1, porta UART
#   1A86:7523  CH340                    devkits alternativos
#   1A86:55D4  CH9102                   devkits alternativos
#   0483:2710  STM32 CDC                receptor LoRa original
#
# A taxa e por dispositivo, e nao e detalhe de estilo: o caminho ESP-NOW
# entrega cerca de 50 records/s de ~430 B com o objeto "link" anexado, perto
# de 170 kbps, que nao cabem nos 115200 que bastavam para os ~2 records/s do
# caminho LoRa. Em USB CDC nativo o host ignora o valor, mas ele precisa
# estar certo quando se usa a ponte serial do devkit.
USB_RECEIVERS: list[tuple[int, int, int, str]] = [
    (0x303A, 0x1001, 921600, "ESP32-S3 (USB nativo)"),
    (0x303A, 0x4001, 921600, "ESP32-S3 (USB nativo)"),
    (0x10C4, 0xEA60, 921600, "ESP32-S3 (ponte CP2102N)"),
    (0x1A86, 0x7523, 921600, "ESP32 (ponte CH340)"),
    (0x1A86, 0x55D4, 921600, "ESP32 (ponte CH9102)"),
    (0x0483, 0x2710, 115200, "Receptor LoRa STM32"),
]

SERIAL_BAUDRATE: int = 921600          # usado apenas se a porta for forcada
SERIAL_READ_TIMEOUT_S: float = 1.0
RECONNECT_DELAY_S: float = 2.0

# Sequence tracking
SEQ_MODULO: int = 2 ** 32
SEQ_WRAPAROUND_WINDOW: int = 100

# UI / Performance
UI_TICK_MS: int = 400
MAX_GRAPH_POINTS: int = 200
MAX_PACKETS_BUFFER: int = 2000
MAX_ALERT_LOG: int = 200

# Filesystem
BASE_DIR: Path = Path(__file__).resolve().parent
CSV_DIR: Path = BASE_DIR / "logs"
CALIBRATION_FILE: Path = BASE_DIR / "config_calibracao.json"
ALERTS_FILE: Path = BASE_DIR / "config_alertas.json"
FILTERS_FILE: Path = BASE_DIR / "config_filtros.json"

# Catálogo de variáveis de telemetria
TELEMETRY_VARIABLES: dict[str, dict[str, str]] = {
    "roda_fr":     {"label": "Roda FR",         "unit": "rpm"},
    "roda_rr":     {"label": "Roda RR",         "unit": "rpm"},
    "motor_rpm":   {"label": "Motor RPM",       "unit": "rpm"},
    "freio_d":     {"label": "Freio Diant.",    "unit": "bar"},
    "freio_t":     {"label": "Freio Tras.",     "unit": "bar"},
    "ax":          {"label": "Aceleração X",    "unit": "g"},
    "ay":          {"label": "Aceleração Y",    "unit": "g"},
    "az":          {"label": "Aceleração Z",    "unit": "g"},
    "gx":          {"label": "Giroscópio X",    "unit": "deg/s"},
    "gy":          {"label": "Giroscópio Y",    "unit": "deg/s"},
    "gz":          {"label": "Giroscópio Z",    "unit": "deg/s"},
    "bateria_mv":  {"label": "Bateria",         "unit": "mV"},
    "motor_temp":  {"label": "Temp. Motor",     "unit": "°C"},
    "cvt_temp":    {"label": "Temp. CVT",       "unit": "°C"},
    "combustivel": {"label": "Combustível",     "unit": "%"},
}
VAR_KEYS: list[str] = list(TELEMETRY_VARIABLES.keys())

# Campos do objeto "link" que o gateway ESP-NOW anexa a cada record. A chave
# curta e a que vai no fio; o nome legivel e o do dashboard. Cada campo e
# achatado no pacote como link_<chave>, e e isso que permite reaproveitar toda
# a maquina de graficos, tiles e CSV ja existente sem adaptacao.
LINK_FIELDS: dict[str, dict[str, str]] = {
    "rssi": {"label": "RSSI",                  "unit": "dBm"},
    "pdr":  {"label": "PDR (janela 1 s)",      "unit": "%"},
    "fr":   {"label": "Quadros entregues",     "unit": ""},
    "dup":  {"label": "Duplicatas",            "unit": ""},
    "gap":  {"label": "Lacunas declaradas",    "unit": ""},
    "ro":   {"label": "Fila de reordenacao",   "unit": "quadros"},
    "retx": {"label": "Retransmissoes",        "unit": ""},
    "drp":  {"label": "Perda por estouro",     "unit": "records"},
    "gu":   {"label": "Quadros abandonados",   "unit": ""},
    "buf":  {"label": "Buffer em uso",         "unit": "quadros"},
    "bmax": {"label": "Buffer total",          "unit": "quadros"},
    "cwnd": {"label": "Janela",                "unit": "quadros"},
    "srtt": {"label": "RTT estimado",          "unit": "ms"},
    "rto":  {"label": "RTO",                   "unit": "ms"},
    "ch":   {"label": "Canal",                 "unit": ""},
    "lr":   {"label": "Long range",            "unit": ""},
    "src":  {"label": "Fonte",                 "unit": ""},
    "hz":   {"label": "Taxa da fonte",         "unit": "Hz"},
    "up":   {"label": "Associado",             "unit": ""},
    "ld":   {"label": "Quedas de link",        "unit": ""},
}
LINK_KEYS: list[str] = list(LINK_FIELDS.keys())
LINK_CSV_COLS: list[str] = [f"link_{k}" for k in LINK_KEYS]

# Paleta dark
COL_BG     = "#0e1116"
COL_PANEL  = "#171c23"
COL_PANEL2 = "#1d242c"
COL_BORDER = "#2a323d"
COL_TEXT   = "#e6edf3"
COL_MUTED  = "#8b949e"
COL_ACCENT = "#4ea1ff"
COL_OK     = "#22d07a"
COL_WARN   = "#f5b942"
COL_ERR    = "#ff4d5e"
COL_PURPLE = "#b48cff"

PLOTLY_TEMPLATE = "plotly_dark"

# Cores cíclicas para traces de gráfico
TRACE_COLORS: list[str] = [
    COL_ACCENT, COL_OK, COL_WARN, COL_PURPLE, COL_ERR,
    "#ff9e64", "#9ece6a", "#7dcfff", "#bb9af7", "#f7768e", "#e0af68",
]

ALERT_OPERATORS: list[str] = [">", "<", ">=", "<=", "=="]

# =============================================================================
# SECTION 2 — FUNÇÕES PURAS
# =============================================================================

def track_sequence(
    last_seq: Optional[int],
    new_seq: int,
    modulo: int = SEQ_MODULO,
    window: int = SEQ_WRAPAROUND_WINDOW,
) -> tuple[int, int, bool]:
    """
    Rastreamento de sequência tolerante a wraparound modulo `modulo`.

    Retorna (new_last_seq, perdidos_neste_passo, foi_reset).
    """
    if last_seq is None:
        return new_seq, 0, False
    diff = (new_seq - last_seq) % modulo
    if diff == 0:
        # Pacote duplicado (provavelmente echo, ignora)
        return last_seq, 0, False
    if diff > modulo // 2:
        # `new_seq` está "atrás" → reset do MCU (ou reordenação severa)
        return new_seq, 0, True
    if diff > window and diff < modulo - window:
        # Salto muito grande → trata como reset
        return new_seq, 0, True
    return new_seq, diff - 1, False


def calibrate_value(raw: float, mult: float, offset: float) -> float:
    """Aplica calibração linear `out = raw * mult + offset`."""
    return raw * mult + offset


def ema_step(prev: Optional[float], new: float, alpha: float) -> float:
    """Passo de Média Móvel Exponencial. `alpha` em (0, 1]; 1.0 = sem filtragem."""
    if prev is None or alpha >= 0.999:
        return new
    return alpha * new + (1.0 - alpha) * prev


def accel_to_attitude(ax: float, ay: float, az: float) -> tuple[float, float]:
    """
    Converte aceleração 3-eixo (g ou m/s²) em pitch/roll em graus.

    Convenção:
      - pitch = rotação em torno do eixo Y (nose up/down)  = atan2(-ax, sqrt(ay²+az²))
      - roll  = rotação em torno do eixo X (lateral tilt)  = atan2(ay, az)
    Estas fórmulas são numericamente robustas (atan2 cobre todos quadrantes).
    Aviso: somente o vetor gravitacional dá medida estática; sob aceleração
    dinâmica há erro — para precisão use fusão giro+accel (Madgwick/Mahony/Kalman).
    """
    try:
        pitch_rad = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        roll_rad = math.atan2(ay, az)
        return math.degrees(pitch_rad), math.degrees(roll_rad)
    except (ValueError, TypeError, ZeroDivisionError):
        return 0.0, 0.0


def evaluate_alert(value: float, operator: str, threshold: float) -> bool:
    """Avalia comparação binária. Operadores inválidos retornam False."""
    if operator == ">":  return value > threshold
    if operator == "<":  return value < threshold
    if operator == ">=": return value >= threshold
    if operator == "<=": return value <= threshold
    if operator == "==": return value == threshold
    return False


# =============================================================================
# SECTION 3 — CALIBRATION MANAGER
# =============================================================================

class CalibrationManager:
    """
    Persistência e aplicação de calibração linear (`out = raw * mult + offset`).
    Thread-safe. Defaults (1.0, 0.0) para qualquer variável ausente do arquivo.
    """

    def __init__(self, path: Path) -> None:
        self._path = path
        self._lock = threading.Lock()
        self._cal: dict[str, dict[str, float]] = {}
        self._load()

    def _load(self) -> None:
        if not self._path.exists():
            self._cal = {k: {"mult": 1.0, "offset": 0.0} for k in VAR_KEYS}
            self._save_unlocked()
            return
        try:
            raw = json.loads(self._path.read_text(encoding="utf-8"))
            self._cal = {
                k: {
                    "mult": float(raw.get(k, {}).get("mult", 1.0)),
                    "offset": float(raw.get(k, {}).get("offset", 0.0)),
                }
                for k in VAR_KEYS
            }
            log.info("Calibração carregada de %s", self._path.name)
        except Exception as e:
            log.error("Falha ao carregar calibração: %s — usando defaults", e)
            self._cal = {k: {"mult": 1.0, "offset": 0.0} for k in VAR_KEYS}

    def _save_unlocked(self) -> None:
        try:
            self._path.write_text(json.dumps(self._cal, indent=2), encoding="utf-8")
        except OSError as e:
            log.error("Falha ao gravar calibração: %s", e)

    def get(self, var: str) -> tuple[float, float]:
        with self._lock:
            d = self._cal.get(var, {"mult": 1.0, "offset": 0.0})
            return d["mult"], d["offset"]

    def all_as_table(self) -> list[dict[str, Any]]:
        with self._lock:
            return [
                {
                    "variavel": k,
                    "label": TELEMETRY_VARIABLES[k]["label"],
                    "unidade": TELEMETRY_VARIABLES[k]["unit"],
                    "mult": self._cal[k]["mult"],
                    "offset": self._cal[k]["offset"],
                }
                for k in VAR_KEYS
            ]

    def update_from_table(self, rows: list[dict[str, Any]]) -> None:
        with self._lock:
            for row in rows:
                var = row.get("variavel")
                if var not in TELEMETRY_VARIABLES:
                    continue
                try:
                    self._cal[var] = {
                        "mult": float(row.get("mult", 1.0)),
                        "offset": float(row.get("offset", 0.0)),
                    }
                except (TypeError, ValueError):
                    log.warning("Calibração inválida para %s: %s", var, row)
            self._save_unlocked()
        log.info("Calibração atualizada e persistida.")


# =============================================================================
# SECTION 4 — EMA FILTER (DSP)
# =============================================================================

class EMAFilter:
    """
    Filtro Passa-Baixa por Média Móvel Exponencial, com alpha configurável
    por variável. `alpha=1.0` desliga o filtro (passthrough).

    Estado persistido por variável; alpha hot-reloadable (sem reset do estado).
    """

    def __init__(self, path: Path) -> None:
        self._path = path
        self._lock = threading.Lock()
        self._alpha: dict[str, float] = {k: 1.0 for k in VAR_KEYS}
        self._state: dict[str, Optional[float]] = {k: None for k in VAR_KEYS}
        self._load()

    def _load(self) -> None:
        if not self._path.exists():
            self._save_unlocked()
            return
        try:
            raw = json.loads(self._path.read_text(encoding="utf-8"))
            for k in VAR_KEYS:
                a = float(raw.get(k, 1.0))
                self._alpha[k] = max(0.001, min(1.0, a))
            log.info("Filtros EMA carregados de %s", self._path.name)
        except Exception as e:
            log.error("Falha ao carregar filtros EMA: %s", e)

    def _save_unlocked(self) -> None:
        try:
            self._path.write_text(json.dumps(self._alpha, indent=2), encoding="utf-8")
        except OSError as e:
            log.error("Falha ao gravar filtros EMA: %s", e)

    def reset_state(self) -> None:
        with self._lock:
            for k in VAR_KEYS:
                self._state[k] = None

    def apply(self, var: str, value: float) -> float:
        with self._lock:
            alpha = self._alpha.get(var, 1.0)
            new = ema_step(self._state.get(var), value, alpha)
            self._state[var] = new
            return new

    def all_as_table(self) -> list[dict[str, Any]]:
        with self._lock:
            return [
                {
                    "variavel": k,
                    "label": TELEMETRY_VARIABLES[k]["label"],
                    "alpha": self._alpha.get(k, 1.0),
                }
                for k in VAR_KEYS
            ]

    def update_from_table(self, rows: list[dict[str, Any]]) -> None:
        with self._lock:
            for row in rows:
                var = row.get("variavel")
                if var not in TELEMETRY_VARIABLES:
                    continue
                try:
                    a = float(row.get("alpha", 1.0))
                    self._alpha[var] = max(0.001, min(1.0, a))
                except (TypeError, ValueError):
                    log.warning("Alpha inválido para %s: %s", var, row)
            self._save_unlocked()
        log.info("Filtros EMA atualizados e persistidos.")


# =============================================================================
# SECTION 5 — ALERT ENGINE
# =============================================================================

@dataclass
class AlertRule:
    id: str
    variable: str
    operator: str
    threshold: float
    message: str
    enabled: bool = True


@dataclass
class AlertEvent:
    timestamp: str
    rule_id: str
    variable: str
    value: float
    threshold: float
    operator: str
    message: str


class AlertEngine:
    """
    Engine de alertas com regras configuráveis. Emite eventos somente na
    borda de subida (transição inativo→ativo) para evitar spam no log.
    """

    def __init__(self, path: Path) -> None:
        self._path = path
        self._lock = threading.Lock()
        self._rules: dict[str, AlertRule] = {}
        self._active: dict[str, bool] = {}
        self._events: deque[AlertEvent] = deque(maxlen=MAX_ALERT_LOG)
        self._load()

    def _load(self) -> None:
        if not self._path.exists():
            self._seed_defaults()
            self._save_unlocked()
            return
        try:
            raw = json.loads(self._path.read_text(encoding="utf-8"))
            self._rules = {}
            for r in raw.get("rules", []):
                rid = str(r.get("id", uuid.uuid4().hex[:8]))
                self._rules[rid] = AlertRule(
                    id=rid,
                    variable=str(r["variable"]),
                    operator=str(r["operator"]),
                    threshold=float(r["threshold"]),
                    message=str(r.get("message", "")),
                    enabled=bool(r.get("enabled", True)),
                )
            log.info("Regras de alerta carregadas: %d", len(self._rules))
        except Exception as e:
            log.error("Falha ao carregar alertas: %s — usando defaults", e)
            self._seed_defaults()

    def _seed_defaults(self) -> None:
        defaults = [
            AlertRule(uuid.uuid4().hex[:8], "cvt_temp",   ">", 110.0, "CVT superaquecendo"),
            AlertRule(uuid.uuid4().hex[:8], "motor_temp", ">", 110.0, "Motor superaquecendo"),
            AlertRule(uuid.uuid4().hex[:8], "bateria_mv", "<", 11500.0, "Bateria baixa"),
            AlertRule(uuid.uuid4().hex[:8], "motor_rpm",  ">", 4200.0, "Overspeed motor"),
        ]
        self._rules = {r.id: r for r in defaults}

    def _save_unlocked(self) -> None:
        try:
            payload = {"rules": [asdict(r) for r in self._rules.values()]}
            self._path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        except OSError as e:
            log.error("Falha ao gravar alertas: %s", e)

    def evaluate(self, packet: dict[str, Any]) -> list[AlertEvent]:
        """Avalia regras contra um pacote. Retorna eventos novos (rising edge)."""
        new_events: list[AlertEvent] = []
        with self._lock:
            for rid, rule in self._rules.items():
                if not rule.enabled:
                    continue
                # Avaliamos sobre valor filtrado (mais coerente com display/decisão)
                key = f"{rule.variable}_filt"
                if key not in packet:
                    key = rule.variable
                if key not in packet or packet[key] is None:
                    continue
                try:
                    val = float(packet[key])
                except (TypeError, ValueError):
                    continue
                triggered = evaluate_alert(val, rule.operator, rule.threshold)
                was_active = self._active.get(rid, False)
                if triggered and not was_active:
                    evt = AlertEvent(
                        timestamp=datetime.now().isoformat(timespec="milliseconds"),
                        rule_id=rid,
                        variable=rule.variable,
                        value=val,
                        threshold=rule.threshold,
                        operator=rule.operator,
                        message=rule.message or f"{rule.variable} {rule.operator} {rule.threshold}",
                    )
                    self._events.append(evt)
                    new_events.append(evt)
                    log.warning("ALERTA: %s (%.3f %s %.3f)",
                                rule.message, val, rule.operator, rule.threshold)
                self._active[rid] = triggered
        return new_events

    def active_count(self) -> int:
        with self._lock:
            return sum(1 for v in self._active.values() if v)

    def active_rules(self) -> list[AlertRule]:
        with self._lock:
            return [self._rules[rid] for rid, a in self._active.items() if a and rid in self._rules]

    def all_as_table(self) -> list[dict[str, Any]]:
        with self._lock:
            return [
                {
                    "id": r.id,
                    "variable": r.variable,
                    "operator": r.operator,
                    "threshold": r.threshold,
                    "message": r.message,
                    "enabled": "sim" if r.enabled else "não",
                }
                for r in self._rules.values()
            ]

    def update_from_table(self, rows: list[dict[str, Any]]) -> None:
        new_rules: dict[str, AlertRule] = {}
        for row in rows:
            try:
                rid = str(row.get("id") or uuid.uuid4().hex[:8])
                var = str(row.get("variable", "")).strip()
                op = str(row.get("operator", ">")).strip()
                if var not in TELEMETRY_VARIABLES:
                    log.warning("Regra ignorada — variável inválida: %s", var)
                    continue
                if op not in ALERT_OPERATORS:
                    log.warning("Regra ignorada — operador inválido: %s", op)
                    continue
                thr = float(row.get("threshold", 0.0))
                msg = str(row.get("message", "")).strip()
                enabled_raw = str(row.get("enabled", "sim")).strip().lower()
                enabled = enabled_raw in ("sim", "true", "1", "yes", "y", "on")
                new_rules[rid] = AlertRule(rid, var, op, thr, msg, enabled)
            except Exception as e:
                log.warning("Linha de alerta inválida ignorada: %s (%s)", row, e)
        with self._lock:
            self._rules = new_rules
            # Limpa estados de regras que não existem mais
            self._active = {rid: a for rid, a in self._active.items() if rid in new_rules}
            self._save_unlocked()
        log.info("Regras de alerta atualizadas: %d total", len(new_rules))

    def events_as_table(self) -> list[dict[str, Any]]:
        with self._lock:
            return [
                {
                    "timestamp": e.timestamp,
                    "variable": e.variable,
                    "condicao": f"{e.operator} {e.threshold:.3f}",
                    "valor": f"{e.value:.3f}",
                    "mensagem": e.message,
                }
                for e in reversed(self._events)
            ]

    def clear_log(self) -> None:
        with self._lock:
            self._events.clear()
        log.info("Log de alertas limpo.")


# =============================================================================
# SECTION 6 — DATA STORE
# =============================================================================

class DataStore:
    """
    Buffer thread-safe de pacotes processados.

    Cada pacote armazenado contém:
      - keys brutas: `var` (ex: 'motor_rpm')
      - keys filtradas: `var_filt` (ex: 'motor_rpm_filt')
      - 'ts'  : timestamp do MCU (ms ou unidade do firmware)
      - 'seq' : sequence number original
      - 'pc_ts': datetime ISO do PC quando o pacote chegou
      - 'estop', 'tele', etc.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._packets: deque[dict[str, Any]] = deque(maxlen=MAX_PACKETS_BUFFER)
        # Estatísticas de sequência
        self._last_seq: Optional[int] = None
        self._lost_count: int = 0
        self._reset_count: int = 0
        self._total_rx: int = 0
        # Última atitude calculada
        self._last_pitch: float = 0.0
        self._last_roll: float = 0.0

    def reset_session(self) -> None:
        with self._lock:
            self._packets.clear()
            self._last_seq = None
            self._lost_count = 0
            self._reset_count = 0
            self._total_rx = 0
            self._last_pitch = 0.0
            self._last_roll = 0.0

    def push(self, packet: dict[str, Any]) -> tuple[int, bool]:
        """
        Insere pacote. Retorna (perdidos_neste_passo, foi_reset).
        Aplica também a atualização de atitude.
        """
        with self._lock:
            seq = packet.get("seq")
            lost = 0
            was_reset = False
            if isinstance(seq, int):
                self._last_seq, lost, was_reset = track_sequence(self._last_seq, seq)
                self._lost_count += lost
                if was_reset:
                    self._reset_count += 1
            self._total_rx += 1
            # Atitude (a partir dos valores filtrados, mais estável visualmente)
            ax = packet.get("ax_filt", packet.get("ax"))
            ay = packet.get("ay_filt", packet.get("ay"))
            az = packet.get("az_filt", packet.get("az"))
            if ax is not None and ay is not None and az is not None:
                self._last_pitch, self._last_roll = accel_to_attitude(
                    float(ax), float(ay), float(az)
                )
            self._packets.append(packet)
        return lost, was_reset

    def snapshot_stats(self) -> dict[str, Any]:
        with self._lock:
            loss_pct = (
                (self._lost_count / (self._lost_count + self._total_rx) * 100.0)
                if (self._lost_count + self._total_rx) > 0
                else 0.0
            )
            last = self._packets[-1] if self._packets else None
            return {
                "total_rx": self._total_rx,
                "lost": self._lost_count,
                "loss_pct": loss_pct,
                "resets": self._reset_count,
                "last_pitch": self._last_pitch,
                "last_roll": self._last_roll,
                "last_packet": last,
            }

    def get_samples_after(
        self,
        var: str,
        last_ts: Optional[float],
        use_filtered: bool = True,
    ) -> list[tuple[float, float]]:
        """Retorna [(mcu_ts, valor)] com ts > last_ts."""
        key_pref = f"{var}_filt" if use_filtered else var
        out: list[tuple[float, float]] = []
        with self._lock:
            for pkt in self._packets:
                ts = pkt.get("ts")
                if ts is None:
                    continue
                if last_ts is not None and ts <= last_ts:
                    continue
                val = pkt.get(key_pref, pkt.get(var))
                if val is None:
                    continue
                try:
                    out.append((float(ts), float(val)))
                except (TypeError, ValueError):
                    continue
        return out

    def get_recent_samples(
        self,
        var: str,
        n: int,
        use_filtered: bool = True,
    ) -> list[tuple[float, float]]:
        """Retorna os últimos n pontos [(mcu_ts, valor)] da variável."""
        key_pref = f"{var}_filt" if use_filtered else var
        out: list[tuple[float, float]] = []
        with self._lock:
            for pkt in self._packets:
                ts = pkt.get("ts")
                if ts is None:
                    continue
                val = pkt.get(key_pref, pkt.get(var))
                if val is None:
                    continue
                try:
                    out.append((float(ts), float(val)))
                except (TypeError, ValueError):
                    continue
        return out[-n:] if len(out) > n else out


# =============================================================================
# SECTION 7 — GRAPH STATE TRACKER (suporte ao dash.Patch)
# =============================================================================

class GraphStateTracker:
    """
    Estado server-side por gráfico/variável para emitir Patches precisos:
      - last_ts:     último timestamp já enviado ao cliente
      - client_len:  tamanho atual do buffer do cliente (para saber quanto trimar)
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_ts: dict[str, dict[str, Optional[float]]] = {}
        self._client_len: dict[str, dict[str, int]] = {}

    def reset(self, graph_id: str) -> None:
        with self._lock:
            self._last_ts[graph_id] = {}
            self._client_len[graph_id] = {}

    def get(self, graph_id: str, var: str) -> tuple[Optional[float], int]:
        with self._lock:
            last_ts = self._last_ts.get(graph_id, {}).get(var)
            client_len = self._client_len.get(graph_id, {}).get(var, 0)
            return last_ts, client_len

    def set(self, graph_id: str, var: str, last_ts: float, client_len: int) -> None:
        with self._lock:
            self._last_ts.setdefault(graph_id, {})[var] = last_ts
            self._client_len.setdefault(graph_id, {})[var] = client_len


# =============================================================================
# SECTION 8 — SERIAL MANAGER
# =============================================================================

class SerialManager:
    """
    Gerencia conexão USB CDC via VID:PID, lê linhas JSON em thread daemon,
    aplica calibração+EMA, alimenta DataStore e CSV. Reconecta em RECONNECT_DELAY_S.
    """

    def __init__(
        self,
        store: DataStore,
        calibration: CalibrationManager,
        ema: EMAFilter,
        alert_engine: AlertEngine,
    ) -> None:
        self._store = store
        self._cal = calibration
        self._ema = ema
        self._alerts = alert_engine
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._connected = threading.Event()
        self._port_name: Optional[str] = None
        self._csv_file: Optional[Any] = None
        self._csv_writer: Optional[csv.DictWriter] = None
        self._csv_path: Optional[Path] = None
        self._csv_lock = threading.Lock()
        self._logging_enabled: bool = False
        self._logging_paused: bool = False
        self._device_kind: Optional[str] = None

    # ---- ciclo de vida -----------------------------------------------------

    def start(self) -> None:
        # Garante que thread anterior terminou antes de criar nova
        if self._thread and self._thread.is_alive():
            self._stop_event.set()
            self._thread.join(timeout=3.0)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="serial-rx", daemon=True)
        self._thread.start()
        log.info("SerialManager iniciado.")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._logging_enabled = False
        self._logging_paused = False
        self._close_csv()
        log.info("SerialManager parado.")

    def is_connected(self) -> bool:
        return self._connected.is_set()

    def port_name(self) -> Optional[str]:
        return self._port_name

    def device_kind(self) -> Optional[str]:
        """Qual receptor esta conectado, para aparecer na barra de status."""
        return self._device_kind

    def csv_path(self) -> Optional[Path]:
        return self._csv_path

    # ---- controle de logging --------------------------------------------------

    def start_logging(self) -> Optional[str]:
        """Abre novo CSV e inicia gravação. Funciona mesmo sem conexão (gravará ao conectar)."""
        self._logging_paused = False
        self._logging_enabled = True
        if self._connected.is_set() and not self._csv_file:
            self._open_csv()
        name = self._csv_path.name if self._csv_path else None
        log.info("Logging habilitado. Arquivo: %s", name or "(aguardando conexão)")
        return name

    def pause_logging(self) -> None:
        self._logging_paused = True
        log.info("Logging CSV pausado.")

    def resume_logging(self) -> None:
        self._logging_paused = False
        log.info("Logging CSV retomado.")

    def stop_logging(self) -> None:
        self._logging_enabled = False
        self._logging_paused = False
        self._close_csv()
        log.info("Logging CSV parado.")

    def is_logging(self) -> bool:
        return self._logging_enabled and not self._logging_paused and self._csv_file is not None

    def is_logging_paused(self) -> bool:
        return self._logging_enabled and self._logging_paused

    # ---- interna -----------------------------------------------------------

    @staticmethod
    def _find_port() -> Optional[tuple[str, int, str]]:
        """Primeira porta presente que casa com USB_RECEIVERS.

        Devolve (porta, baud, descricao). A ordem de USB_RECEIVERS define a
        precedencia: o gateway ESP-NOW vem antes do receptor LoRa, de modo que
        com os dois plugados o dashboard abre o caminho novo.
        """
        for p in serial.tools.list_ports.comports():
            for vid, pid, baud, kind in USB_RECEIVERS:
                if p.vid == vid and p.pid == pid:
                    return p.device, baud, kind
        return None

    def _open_csv(self) -> None:
        CSV_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = CSV_DIR / f"telemetria_{ts}.csv"
        fieldnames = ["pc_ts", "ts", "seq"] + VAR_KEYS + [
            "estop", "tele_rx", "tele_lost", "tele_loss_pct", "tele_rst",
        ] + LINK_CSV_COLS
        # newline='' garante quebras de linha corretas no Windows
        self._csv_file = self._csv_path.open("w", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=fieldnames)
        self._csv_writer.writeheader()
        self._csv_file.flush()
        log.info("CSV aberto: %s", self._csv_path.name)

    def _close_csv(self) -> None:
        with self._csv_lock:
            if self._csv_file:
                try:
                    self._csv_file.flush()
                    self._csv_file.close()
                except OSError as e:
                    log.warning("Erro ao fechar CSV: %s", e)
            self._csv_file = None
            self._csv_writer = None

    def _write_csv(self, packet: dict[str, Any]) -> None:
        if not self._logging_enabled or self._logging_paused:
            return
        if not self._csv_writer or not self._csv_file:
            return
        try:
            tele = packet.get("tele") or {}
            row = {
                "pc_ts": packet.get("pc_ts", ""),
                "ts":    packet.get("ts", ""),
                "seq":   packet.get("seq", ""),
                "estop": packet.get("estop", ""),
                "tele_rx":      tele.get("rx", ""),
                "tele_lost":    tele.get("lost", ""),
                "tele_loss_pct": tele.get("loss_pct", ""),
                "tele_rst":     tele.get("rst", ""),
            }
            for k in VAR_KEYS:
                # CSV grava SEMPRE valores brutos (sem calibração, sem EMA)
                row[k] = packet.get(f"{k}_raw", packet.get(k, ""))
            # Colunas do link: ficam vazias em logs do caminho LoRa, o que
            # mantem os CSVs antigos e novos lidos pelo mesmo playback.
            for k in LINK_KEYS:
                row[f"link_{k}"] = packet.get(f"link_{k}", "")
            with self._csv_lock:
                self._csv_writer.writerow(row)
                self._csv_file.flush()
        except Exception as e:
            log.warning("Falha ao escrever linha CSV: %s", e)

    def _process_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        try:
            raw = json.loads(line)
            if not isinstance(raw, dict):
                return
        except json.JSONDecodeError:
            # JSON malformado: descarta silenciosamente (firmware pode emitir
            # frames truncados durante boot/reset)
            return

        # Constrói pacote enriquecido
        packet: dict[str, Any] = {}
        packet["pc_ts"] = datetime.now().isoformat(timespec="milliseconds")
        for k in ("ts", "seq", "estop", "tele"):
            if k in raw:
                packet[k] = raw[k]

        # Saude do link ESP-NOW. Guardada como dicionario para a barra de
        # status e os tiles, e tambem achatada em link_<campo> para que
        # get_recent_samples() e o CSV a tratem como qualquer outra serie
        # numerica. Records do receptor LoRa antigo simplesmente nao tem o
        # objeto, e nesse caso nada disso aparece -- o dashboard continua
        # funcionando com o "tele" de sempre.
        link = raw.get("link")
        if isinstance(link, dict):
            packet["link"] = link
            for k in LINK_KEYS:
                v = link.get(k)
                if v is None:
                    continue
                try:
                    packet[f"link_{k}"] = float(v)
                except (TypeError, ValueError):
                    continue

        # Para cada variável: armazena bruto, calibrado, e filtrado
        for var in VAR_KEYS:
            if var not in raw:
                continue
            try:
                raw_val = float(raw[var])
            except (TypeError, ValueError):
                continue
            mult, offset = self._cal.get(var)
            calibrated = calibrate_value(raw_val, mult, offset)
            filtered = self._ema.apply(var, calibrated)
            packet[f"{var}_raw"] = raw_val
            packet[var] = calibrated
            packet[f"{var}_filt"] = filtered

        # Persiste no CSV (raw)
        self._write_csv(packet)

        # Atualiza buffer + estatísticas
        self._store.push(packet)

        # Avalia regras de alerta
        self._alerts.evaluate(packet)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            found = self._find_port()
            if found is None:
                self._connected.clear()
                self._port_name = None
                self._device_kind = None
                time.sleep(RECONNECT_DELAY_S)
                continue

            port, baud, kind = found
            self._port_name = port
            self._device_kind = kind
            try:
                ser = serial.Serial(port, baud, timeout=SERIAL_READ_TIMEOUT_S)
            except (serial.SerialException, OSError) as e:
                log.warning("Falha ao abrir %s: %s", port, e)
                time.sleep(RECONNECT_DELAY_S)
                continue

            log.info("Conectado em %s (%s, %d baud)", port, kind, baud)
            if self._logging_enabled:
                self._open_csv()
            self._connected.set()

            try:
                while not self._stop_event.is_set():
                    try:
                        line_bytes = ser.readline()
                    except (serial.SerialException, OSError) as e:
                        log.warning("Erro de leitura: %s — reconectando", e)
                        break
                    if not line_bytes:
                        continue
                    try:
                        line = line_bytes.decode("utf-8", errors="ignore")
                    except Exception:
                        continue
                    try:
                        self._process_line(line)
                    except Exception as e:
                        # Nunca deixa exceção derrubar a thread
                        log.error("Erro processando linha: %s", e)
            finally:
                self._connected.clear()
                try:
                    ser.close()
                except OSError:
                    pass
                self._close_csv()
                self._device_kind = None
                log.info("Desconectado de %s", port)

            if not self._stop_event.is_set():
                time.sleep(RECONNECT_DELAY_S)


# =============================================================================
# SECTION 9 — PLAYBACK MANAGER
# =============================================================================

class PlaybackManager:
    """
    Reprodução de CSV respeitando delta de tempo entre pacotes (ts do MCU em ms).
    Play/Pause/Stop + multiplicador de velocidade (0.5x .. 10x).
    """

    def __init__(
        self,
        store: DataStore,
        calibration: CalibrationManager,
        ema: EMAFilter,
        alert_engine: AlertEngine,
    ) -> None:
        self._store = store
        self._cal = calibration
        self._ema = ema
        self._alerts = alert_engine
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._lock = threading.Lock()
        self._speed: float = 1.0
        self._csv_path: Optional[Path] = None
        self._running = False
        self._progress = 0.0

    def is_running(self) -> bool:
        return self._running

    def is_paused(self) -> bool:
        return self._pause_event.is_set() and self._running

    def progress(self) -> float:
        return self._progress

    def csv_path(self) -> Optional[Path]:
        return self._csv_path

    def set_speed(self, speed: float) -> None:
        with self._lock:
            self._speed = max(0.1, min(20.0, float(speed)))
        log.info("Playback speed = %.2fx", self._speed)

    def start(self, csv_path: Path, speed: float = 1.0) -> bool:
        if self._thread and self._thread.is_alive():
            self.stop()
        if not csv_path.exists():
            log.error("Arquivo de playback não encontrado: %s", csv_path)
            return False
        self._csv_path = csv_path
        self._speed = max(0.1, min(20.0, float(speed)))
        self._stop_event.clear()
        self._pause_event.clear()
        self._ema.reset_state()
        self._store.reset_session()
        self._running = True
        self._progress = 0.0
        self._thread = threading.Thread(target=self._run, name="playback", daemon=True)
        self._thread.start()
        log.info("Playback iniciado: %s (%.2fx)", csv_path.name, self._speed)
        return True

    def pause(self) -> None:
        self._pause_event.set()
        log.info("Playback pausado.")

    def resume(self) -> None:
        self._pause_event.clear()
        log.info("Playback retomado.")

    def stop(self) -> None:
        self._stop_event.set()
        self._pause_event.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._running = False
        log.info("Playback parado.")

    def _run(self) -> None:
        try:
            with self._csv_path.open("r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                rows = list(reader)
            n = len(rows)
            if n == 0:
                log.warning("CSV de playback vazio: %s", self._csv_path)
                self._running = False
                return
            prev_ts: Optional[float] = None
            for idx, row in enumerate(rows):
                if self._stop_event.is_set():
                    break
                # Pausa cooperativa
                while self._pause_event.is_set() and not self._stop_event.is_set():
                    time.sleep(0.05)
                if self._stop_event.is_set():
                    break

                # Delta de tempo
                try:
                    ts = float(row.get("ts", 0))
                except (TypeError, ValueError):
                    ts = 0.0
                if prev_ts is not None and ts > prev_ts:
                    delta_ms = ts - prev_ts
                    with self._lock:
                        speed = self._speed
                    sleep_s = (delta_ms / 1000.0) / speed
                    # Sleep em chunks para responder a stop rapidamente
                    end = time.monotonic() + sleep_s
                    while time.monotonic() < end:
                        if self._stop_event.is_set():
                            return
                        time.sleep(min(0.05, end - time.monotonic()))
                prev_ts = ts

                # Constrói pacote reprocessado a partir do raw do CSV
                packet: dict[str, Any] = {
                    "pc_ts": datetime.now().isoformat(timespec="milliseconds"),
                }
                for k in ("ts", "seq", "estop"):
                    v = row.get(k)
                    if v not in (None, ""):
                        try:
                            packet[k] = int(float(v)) if k in ("seq", "estop") else float(v)
                        except (TypeError, ValueError):
                            pass
                # tele reconstruído
                tele: dict[str, Any] = {}
                for src, dst in (("tele_rx", "rx"), ("tele_lost", "lost"),
                                 ("tele_loss_pct", "loss_pct"), ("tele_rst", "rst")):
                    v = row.get(src)
                    if v not in (None, ""):
                        try:
                            tele[dst] = float(v) if dst == "loss_pct" else int(float(v))
                        except (TypeError, ValueError):
                            pass
                if tele:
                    packet["tele"] = tele

                # Variáveis: raw -> calibração -> EMA (recomputa do zero, podendo
                # usar calibração/alpha NOVOS — esse é o ganho do playback)
                for var in VAR_KEYS:
                    v = row.get(var)
                    if v in (None, ""):
                        continue
                    try:
                        raw_val = float(v)
                    except (TypeError, ValueError):
                        continue
                    mult, offset = self._cal.get(var)
                    calibrated = calibrate_value(raw_val, mult, offset)
                    filtered = self._ema.apply(var, calibrated)
                    packet[f"{var}_raw"] = raw_val
                    packet[var] = calibrated
                    packet[f"{var}_filt"] = filtered

                self._store.push(packet)
                self._alerts.evaluate(packet)
                self._progress = (idx + 1) / n

            self._progress = 1.0
            log.info("Playback concluído.")
        except Exception as e:
            log.error("Erro no playback: %s", e)
        finally:
            self._running = False


# =============================================================================
# SECTION 10 — FIGURAS PLOTLY
# =============================================================================

def make_empty_main_figure(selected_vars: list[str]) -> go.Figure:
    """Cria figura vazia com uma trace por variável selecionada."""
    fig = go.Figure()
    for i, var in enumerate(selected_vars):
        meta = TELEMETRY_VARIABLES.get(var, {"label": var, "unit": ""})
        color = TRACE_COLORS[i % len(TRACE_COLORS)]
        fig.add_trace(go.Scattergl(
            x=[], y=[],
            mode="lines",
            name=f"{meta['label']} [{meta['unit']}]",
            line=dict(color=color, width=2),
            hovertemplate="%{x}<br>%{y:.3f}<extra>" + meta["label"] + "</extra>",
        ))
    fig.update_layout(
        template=PLOTLY_TEMPLATE,
        paper_bgcolor=COL_BG,
        plot_bgcolor=COL_PANEL,
        margin=dict(l=50, r=20, t=30, b=40),
        xaxis=dict(title="ts (MCU, ms)", gridcolor=COL_BORDER, zerolinecolor=COL_BORDER),
        yaxis=dict(title="Valor", gridcolor=COL_BORDER, zerolinecolor=COL_BORDER),
        legend=dict(bgcolor="rgba(0,0,0,0)", bordercolor=COL_BORDER, borderwidth=1),
        uirevision="main-graph",
        height=460,
    )
    return fig


def build_attitude_figure(pitch_deg: float, roll_deg: float) -> go.Figure:
    """Renderização 3D do chassi inclinado pelo pitch/roll calculados."""
    L = 2.0
    W = 1.2

    # Corners no frame do corpo (z=0 = plano do chassi)
    corners_body = [
        (-L / 2, -W / 2, 0.0),
        ( L / 2, -W / 2, 0.0),
        ( L / 2,  W / 2, 0.0),
        (-L / 2,  W / 2, 0.0),
    ]

    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)

    def rot(pt: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z = pt
        # Pitch: rotação em Y
        x1 = cp * x + sp * z
        y1 = y
        z1 = -sp * x + cp * z
        # Roll: rotação em X
        x2 = x1
        y2 = cr * y1 - sr * z1
        z2 = sr * y1 + cr * z1
        return x2, y2, z2

    rotated = [rot(c) for c in corners_body]
    xs = [c[0] for c in rotated] + [rotated[0][0]]
    ys = [c[1] for c in rotated] + [rotated[0][1]]
    zs = [c[2] for c in rotated] + [rotated[0][2]]

    nose_body = [(0.0, 0.0, 0.05), (L / 2 + 0.3, 0.0, 0.05)]
    nose_rot = [rot(pt) for pt in nose_body]

    fig = go.Figure()

    # Chassi (Mesh3d)
    fig.add_trace(go.Mesh3d(
        x=[c[0] for c in rotated],
        y=[c[1] for c in rotated],
        z=[c[2] for c in rotated],
        i=[0, 0], j=[1, 2], k=[2, 3],
        color=COL_ACCENT, opacity=0.65, name="Chassi", showlegend=False,
        hoverinfo="skip",
    ))

    # Contorno
    fig.add_trace(go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="lines",
        line=dict(color=COL_TEXT, width=5),
        showlegend=False, hoverinfo="skip",
    ))

    # Nariz (frente do carro)
    fig.add_trace(go.Scatter3d(
        x=[nose_rot[0][0], nose_rot[1][0]],
        y=[nose_rot[0][1], nose_rot[1][1]],
        z=[nose_rot[0][2], nose_rot[1][2]],
        mode="lines+markers",
        line=dict(color=COL_OK, width=8),
        marker=dict(size=[3, 8], color=COL_OK),
        showlegend=False, hoverinfo="skip",
    ))

    # Plano de referência (solo)
    fig.add_trace(go.Mesh3d(
        x=[-2.0,  2.0, 2.0, -2.0],
        y=[-2.0, -2.0, 2.0,  2.0],
        z=[-1.3, -1.3, -1.3, -1.3],
        i=[0, 0], j=[1, 2], k=[2, 3],
        color=COL_PANEL2, opacity=0.35, showlegend=False, hoverinfo="skip",
    ))

    fig.update_layout(
        template=PLOTLY_TEMPLATE,
        scene=dict(
            xaxis=dict(range=[-2.2, 2.2], title="X (frente)", backgroundcolor=COL_BG,
                       gridcolor=COL_BORDER, zerolinecolor=COL_BORDER),
            yaxis=dict(range=[-2.2, 2.2], title="Y (lateral)", backgroundcolor=COL_BG,
                       gridcolor=COL_BORDER, zerolinecolor=COL_BORDER),
            zaxis=dict(range=[-1.5, 1.5], title="Z", backgroundcolor=COL_BG,
                       gridcolor=COL_BORDER, zerolinecolor=COL_BORDER),
            aspectmode="cube",
            bgcolor=COL_BG,
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.0)),
        ),
        margin=dict(l=0, r=0, t=10, b=0),
        paper_bgcolor=COL_BG,
        showlegend=False,
        uirevision="attitude-fig",
        height=380,
    )
    return fig


# =============================================================================
# SECTION 11 — UI HELPERS / STYLES
# =============================================================================

STYLE_PAGE = {
    "backgroundColor": COL_BG, "color": COL_TEXT,
    "fontFamily": "system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif",
    "minHeight": "100vh", "padding": "0 16px 24px 16px",
}
STYLE_PANEL = {
    "backgroundColor": COL_PANEL, "border": f"1px solid {COL_BORDER}",
    "borderRadius": "8px", "padding": "14px", "marginBottom": "14px",
}
STYLE_PANEL_TIGHT = {**STYLE_PANEL, "padding": "10px 14px"}
STYLE_LABEL = {"color": COL_MUTED, "fontSize": "12px", "marginBottom": "4px",
               "textTransform": "uppercase", "letterSpacing": "0.5px"}
STYLE_VALUE = {"color": COL_TEXT, "fontSize": "20px", "fontWeight": 600}
STYLE_PILL_BASE = {"padding": "4px 10px", "borderRadius": "999px",
                   "fontSize": "12px", "fontWeight": 600, "display": "inline-block"}


def make_pill(text: str, bg: str, fg: str = COL_BG, extra_style: dict | None = None) -> html.Span:
    style = {**STYLE_PILL_BASE, "backgroundColor": bg, "color": fg}
    if extra_style:
        style.update(extra_style)
    return html.Span(text, style=style)


def loss_color(loss_pct: float) -> str:
    if loss_pct < 1.0:
        return COL_OK
    if loss_pct < 5.0:
        return COL_WARN
    return COL_ERR


def rssi_color(rssi_dbm: int) -> str:
    """Faixas praticas para ESP-NOW em modo long range.

    Acima de -75 dBm o link tem folga. Entre -75 e -90 ainda funciona, mas e
    onde a retransmissao comeca a aparecer. Abaixo de -90 esta no limite da
    sensibilidade e a queda e questao de alguns metros.
    """
    if rssi_dbm == 0:
        return COL_PANEL2
    if rssi_dbm > -75:
        return COL_OK
    if rssi_dbm > -90:
        return COL_WARN
    return COL_ERR


def build_link_figure(
    rssi: list[tuple[float, float]],
    pdr: list[tuple[float, float]],
    buf_pct: list[tuple[float, float]],
) -> go.Figure:
    """RSSI no eixo esquerdo; PDR e ocupacao do buffer no eixo direito.

    As duas series percentuais compartilham o eixo da direita de proposito:
    lidas juntas elas contam a historia do store-and-forward. Quando o RSSI
    cai, o PDR cai atras dele e o buffer sobe -- o sensor esta guardando o que
    nao conseguiu entregar. Quando o link volta, o buffer desce mais rapido
    do que subiu, porque a janela de congestionamento abre acima do regime
    permanente para drenar o atraso. Se o buffer encosta em 100 % e fica, a
    queda durou mais que a autonomia do buffer e ha perda definitiva.
    """
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=[x for x, _ in rssi], y=[y for _, y in rssi],
        name="RSSI (dBm)", mode="lines",
        line={"color": COL_ACCENT, "width": 2},
    ))
    fig.add_trace(go.Scatter(
        x=[x for x, _ in pdr], y=[y for _, y in pdr],
        name="PDR (%)", mode="lines", yaxis="y2",
        line={"color": COL_OK, "width": 2},
    ))
    fig.add_trace(go.Scatter(
        x=[x for x, _ in buf_pct], y=[y for _, y in buf_pct],
        name="Buffer do sensor (% da capacidade)", mode="lines", yaxis="y2",
        line={"color": COL_WARN, "width": 1.6, "dash": "dot"},
    ))
    fig.update_layout(
        template=PLOTLY_TEMPLATE,
        paper_bgcolor=COL_PANEL, plot_bgcolor=COL_PANEL,
        font={"color": COL_TEXT},
        margin={"l": 58, "r": 58, "t": 8, "b": 42},
        height=290,
        xaxis={"title": "ts do MCU (ms)", "gridcolor": COL_BORDER,
               "zerolinecolor": COL_BORDER},
        yaxis={"title": "RSSI (dBm)", "gridcolor": COL_BORDER,
               "zerolinecolor": COL_BORDER},
        yaxis2={"title": "PDR / buffer (%)", "overlaying": "y", "side": "right",
                "range": [0, 105], "showgrid": False},
        legend={"orientation": "h", "y": 1.16, "x": 0,
                "bgcolor": "rgba(0,0,0,0)"},
        hovermode="x unified",
        uirevision="link-panel",
    )
    return fig


def _link_tile(label: str, value: str, color: str = COL_TEXT) -> html.Div:
    return html.Div([
        html.Div(label, style=STYLE_LABEL),
        html.Div(value, style={**STYLE_VALUE, "color": color, "fontSize": "18px"}),
    ], style={"backgroundColor": COL_PANEL2, "border": f"1px solid {COL_BORDER}",
              "borderRadius": "6px", "padding": "10px 12px"})


def build_link_tiles(link: dict[str, Any]) -> list[Any]:
    """Cartoes de saude do link a partir do ultimo record recebido."""
    if not link:
        return [html.Div(
            "Sem objeto \"link\" nos records. Isto e esperado no caminho LoRa "
            "antigo ou em playback de logs anteriores a v3.",
            style={"color": COL_MUTED, "fontSize": "13px", "gridColumn": "1 / -1"},
        )]

    def i(k: str) -> int:
        try:
            return int(float(link.get(k, 0) or 0))
        except (TypeError, ValueError):
            return 0

    def f(k: str) -> float:
        try:
            return float(link.get(k, 0.0) or 0.0)
        except (TypeError, ValueError):
            return 0.0

    pdr = f("pdr")
    buf, bmax = i("buf"), i("bmax")
    bpct = (100.0 * buf / bmax) if bmax else 0.0
    frames, gaps, dups = i("fr"), i("gap"), i("dup")
    retx = i("retx")
    sent = frames + retx

    return [
        _link_tile("Associado", "sim" if i("up") else "nao",
                   COL_OK if i("up") else COL_ERR),
        _link_tile("PDR (janela 1 s)", f"{pdr:.2f} %", loss_color(100.0 - pdr)),
        _link_tile("RSSI", f"{i('rssi')} dBm", rssi_color(i("rssi"))),
        _link_tile("Buffer do sensor", f"{buf} / {bmax}",
                   COL_OK if bpct < 25 else (COL_WARN if bpct < 70 else COL_ERR)),
        _link_tile("Retransmissoes",
                   f"{retx}" + (f"  ({100.0 * retx / sent:.1f} %)" if sent else ""),
                   COL_TEXT if not sent or retx / sent < 0.10 else COL_WARN),
        _link_tile("Lacunas declaradas", f"{gaps}",
                   COL_OK if gaps == 0 else COL_WARN),
        _link_tile("Duplicatas", f"{dups}"),
        _link_tile("Perda definitiva",
                   f"{i('drp')} est. / {i('gu')} praz.",
                   COL_OK if (i("drp") + i("gu")) == 0 else COL_ERR),
        _link_tile("RTT / RTO", f"{i('srtt')} / {i('rto')} ms"),
        _link_tile("Janela", f"{i('cwnd')} quadros"),
        _link_tile("Canal / PHY",
                   f"{i('ch')}" + ("  LR" if i("lr") else "  11b/g/n")),
        _link_tile("Fonte",
                   ("UART" if i("src") else "emulador") + f"  {i('hz')} Hz"),
        _link_tile("Fila de reordenacao", f"{i('ro')} quadros"),
        _link_tile("Quedas de link", f"{i('ld')}",
                   COL_TEXT if i("ld") == 0 else COL_WARN),
    ]


# =============================================================================
# SECTION 12 — APP (instâncias globais)
# =============================================================================

CSV_DIR.mkdir(parents=True, exist_ok=True)

calibration_mgr = CalibrationManager(CALIBRATION_FILE)
ema_filter = EMAFilter(FILTERS_FILE)
alert_engine = AlertEngine(ALERTS_FILE)
data_store = DataStore()
graph_tracker = GraphStateTracker()
serial_mgr = SerialManager(data_store, calibration_mgr, ema_filter, alert_engine)
playback_mgr = PlaybackManager(data_store, calibration_mgr, ema_filter, alert_engine)

# Por padrão, inicia a thread serial em modo "live"
serial_mgr.start()

app = Dash(
    __name__,
    title="Telemetria Baja SAE",
    suppress_callback_exceptions=True,
    update_title=None,
)

# CSS custom (animação blink p/ alertas e E-STOP)
app.index_string = """
<!DOCTYPE html>
<html>
<head>
    {%metas%}
    <title>{%title%}</title>
    {%favicon%}
    {%css%}
    <style>
        body { background-color: """ + COL_BG + """; margin: 0; }
        @keyframes blink {
            0%, 100% { opacity: 1.0; }
            50%      { opacity: 0.35; }
        }
        .blink-alert { animation: blink 0.8s infinite; }
        .tab-content { padding-top: 12px; }
        .dash-table-container .dash-spreadsheet-container .dash-spreadsheet-inner table {
            background-color: """ + COL_PANEL + """ !important;
        }
        .dash-table-container .dash-header { background-color: """ + COL_PANEL2 + """ !important; }
    </style>
</head>
<body>
    {%app_entry%}
    <footer>{%config%}{%scripts%}{%renderer%}</footer>
</body>
</html>
"""

# =============================================================================
# SECTION 13 — LAYOUT
# =============================================================================

def build_dashboard_tab() -> html.Div:
    return html.Div([
        # Controles de fonte
        html.Div([
            html.Div([
                html.Div("Fonte de dados", style=STYLE_LABEL),
                dcc.RadioItems(
                    id="src-radio",
                    options=[
                        {"label": " Live (USB)",  "value": "live"},
                        {"label": " Playback (CSV)", "value": "playback"},
                    ],
                    value="live", inline=True,
                    style={"color": COL_TEXT},
                    inputStyle={"marginRight": "6px", "marginLeft": "12px"},
                ),
            ], style={"flex": "1"}),

            html.Div([
                html.Div("Variáveis no gráfico", style=STYLE_LABEL),
                dcc.Dropdown(
                    id="selected-vars",
                    options=[{"label": f"{v['label']} ({v['unit']})", "value": k}
                             for k, v in TELEMETRY_VARIABLES.items()],
                    value=["motor_rpm", "cvt_temp"],
                    multi=True,
                    style={"minWidth": "320px", "color": "#000"},
                ),
            ], style={"flex": "2"}),
        ], style={**STYLE_PANEL, "display": "flex", "gap": "16px", "alignItems": "flex-start"}),

        # Painel de controle de logging live (visibilidade controlada por callback)
        html.Div(id="live-logging-panel", style=STYLE_PANEL, children=[
            html.Div("Logging CSV", style={**STYLE_LABEL, "fontSize": "13px"}),
            html.Div([
                html.Button("⏺ Iniciar log", id="log-start", n_clicks=0,
                            style={"backgroundColor": COL_ERR, "color": COL_TEXT, "border": "none",
                                   "padding": "8px 14px", "borderRadius": "6px",
                                   "cursor": "pointer", "fontWeight": 600}),
                html.Button("⏸ Pausar log", id="log-pause", n_clicks=0,
                            style={"backgroundColor": COL_WARN, "color": COL_BG, "border": "none",
                                   "padding": "8px 14px", "borderRadius": "6px",
                                   "cursor": "pointer", "fontWeight": 600}),
                html.Button("⏹ Parar log", id="log-stop", n_clicks=0,
                            style={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                                   "border": f"1px solid {COL_BORDER}",
                                   "padding": "8px 14px", "borderRadius": "6px",
                                   "cursor": "pointer", "fontWeight": 600}),
                html.Span(id="log-status", children="Log não iniciado.",
                          style={"marginLeft": "4px", "color": COL_MUTED, "fontSize": "13px"}),
            ], style={"display": "flex", "gap": "8px", "alignItems": "center"}),
        ]),

        # Painel playback (visibilidade controlada por callback)
        html.Div(id="playback-panel", style={**STYLE_PANEL, "display": "none"}, children=[
            html.Div("Playback", style={**STYLE_LABEL, "fontSize": "13px"}),
            # Seleção rápida dos logs disponíveis
            html.Div([
                dcc.Dropdown(
                    id="pb-log-dropdown",
                    options=[],
                    placeholder="Selecione um log da pasta logs/...",
                    style={"flex": "1", "color": "#000"},
                ),
                html.Button("🔄", id="pb-refresh-logs", n_clicks=0, title="Atualizar lista",
                            style={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                                   "border": f"1px solid {COL_BORDER}",
                                   "padding": "8px 12px", "borderRadius": "6px",
                                   "cursor": "pointer", "fontWeight": 600}),
            ], style={"display": "flex", "gap": "8px", "alignItems": "center",
                      "marginBottom": "8px"}),
            # Campo manual + botões de controle
            html.Div([
                dcc.Input(
                    id="playback-path", type="text",
                    placeholder="Ou informe o caminho completo do CSV...",
                    style={"flex": "1", "padding": "8px", "backgroundColor": COL_PANEL2,
                           "color": COL_TEXT, "border": f"1px solid {COL_BORDER}",
                           "borderRadius": "6px"},
                ),
                html.Button("▶ Play",  id="pb-play",   n_clicks=0,
                            style={"backgroundColor": COL_OK, "color": COL_BG, "border": "none",
                                   "padding": "8px 14px", "borderRadius": "6px",
                                   "cursor": "pointer", "fontWeight": 600}),
                html.Button("⏸ Pause", id="pb-pause",  n_clicks=0,
                            style={"backgroundColor": COL_WARN, "color": COL_BG, "border": "none",
                                   "padding": "8px 14px", "borderRadius": "6px",
                                   "cursor": "pointer", "fontWeight": 600}),
                html.Button("⏹ Stop",  id="pb-stop",   n_clicks=0,
                            style={"backgroundColor": COL_ERR, "color": COL_TEXT, "border": "none",
                                   "padding": "8px 14px", "borderRadius": "6px",
                                   "cursor": "pointer", "fontWeight": 600}),
                dcc.Dropdown(
                    id="pb-speed",
                    options=[{"label": f"{s}x", "value": s} for s in (0.5, 1, 2, 5, 10)],
                    value=1, clearable=False,
                    style={"width": "100px", "color": "#000"},
                ),
            ], style={"display": "flex", "gap": "8px", "alignItems": "center"}),
            html.Div(id="playback-status", style={"marginTop": "8px", "color": COL_MUTED,
                                                   "fontSize": "13px"}),
        ]),

        # Tiles numéricos
        html.Div(id="tiles-container", style={
            "display": "grid",
            "gridTemplateColumns": "repeat(auto-fit, minmax(160px, 1fr))",
            "gap": "10px", "marginBottom": "14px",
        }),

        # Painel de saude do link ESP-NOW
        html.Div([
            html.Div("Link ESP-NOW (BajaLink)",
                     style={**STYLE_LABEL, "fontSize": "13px"}),
            html.Div(
                "Entrega, alcance e comportamento do store-and-forward. Os "
                "contadores sao cumulativos desde o boot do gateway; o PDR e "
                "em janela movel de 1 s.",
                style={"color": COL_MUTED, "fontSize": "12px",
                       "marginBottom": "10px"},
            ),
            html.Div(id="link-tiles", style={
                "display": "grid",
                "gridTemplateColumns": "repeat(auto-fit, minmax(150px, 1fr))",
                "gap": "8px", "marginBottom": "12px",
            }),
            dcc.Graph(id="link-graph",
                      figure=build_link_figure([], [], []),
                      config={"displaylogo": False}),
        ], style=STYLE_PANEL),

        # Gráfico principal
        html.Div([
            dcc.Graph(
                id="main-graph",
                figure=make_empty_main_figure(["motor_rpm", "cvt_temp"]),
                config={"displaylogo": False},
            ),
        ], style=STYLE_PANEL),

        # Painel de atitude 3D
        html.Div([
            html.Div("Atitude 3D (Pitch/Roll a partir do acelerômetro)",
                     style={**STYLE_LABEL, "fontSize": "13px"}),
            html.Div([
                html.Div([
                    html.Div("Pitch", style=STYLE_LABEL),
                    html.Div(id="attitude-pitch-val", style=STYLE_VALUE),
                ], style={"marginRight": "20px"}),
                html.Div([
                    html.Div("Roll", style=STYLE_LABEL),
                    html.Div(id="attitude-roll-val", style=STYLE_VALUE),
                ]),
            ], style={"display": "flex", "marginBottom": "8px"}),
            dcc.Graph(id="attitude-graph", figure=build_attitude_figure(0.0, 0.0),
                      config={"displaylogo": False}),
        ], style=STYLE_PANEL),
    ])


def build_config_tab() -> html.Div:
    return html.Div([
        # Calibração
        html.Div([
            html.H3("Calibração linear (Valor = Bruto × Mult + Offset)",
                    style={"marginTop": 0, "color": COL_TEXT}),
            html.Div(
                "O CSV grava sempre os valores BRUTOS. A calibração é aplicada apenas no display "
                "e no playback — você pode re-calibrar dados antigos sem perda.",
                style={"color": COL_MUTED, "fontSize": "13px", "marginBottom": "10px"},
            ),
            dash_table.DataTable(
                id="cal-table",
                columns=[
                    {"name": "Variável", "id": "variavel", "editable": False},
                    {"name": "Descrição", "id": "label",   "editable": False},
                    {"name": "Unidade",  "id": "unidade", "editable": False},
                    {"name": "Mult",     "id": "mult",    "type": "numeric", "editable": True},
                    {"name": "Offset",   "id": "offset",  "type": "numeric", "editable": True},
                ],
                data=calibration_mgr.all_as_table(),
                style_cell={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                            "border": f"1px solid {COL_BORDER}", "padding": "8px"},
                style_header={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                              "fontWeight": 600},
                style_data_conditional=[{
                    "if": {"column_editable": True},
                    "backgroundColor": COL_PANEL2,
                }],
            ),
            html.Div([
                html.Button("💾 Salvar calibração", id="cal-save", n_clicks=0,
                            style={"backgroundColor": COL_ACCENT, "color": COL_BG,
                                   "border": "none", "padding": "8px 14px",
                                   "borderRadius": "6px", "cursor": "pointer",
                                   "fontWeight": 600, "marginTop": "10px"}),
                html.Span(id="cal-save-feedback", style={"marginLeft": "12px",
                                                          "color": COL_MUTED}),
            ]),
        ], style=STYLE_PANEL),

        # Filtros EMA
        html.Div([
            html.H3("Filtro EMA (Exponential Moving Average)",
                    style={"marginTop": 0, "color": COL_TEXT}),
            html.Div([
                "Filtro passa-baixa de 1ª ordem: ",
                html.Code("y[n] = α·x[n] + (1−α)·y[n−1]", style={"color": COL_ACCENT}),
                ". ",
                html.B("α=1.0", style={"color": COL_TEXT}),
                " desliga o filtro (passthrough). Valores baixos (ex. 0.1) suavizam ",
                "fortemente sinais ruidosos como RPM e acelerômetros.",
            ], style={"color": COL_MUTED, "fontSize": "13px", "marginBottom": "10px"}),
            dash_table.DataTable(
                id="ema-table",
                columns=[
                    {"name": "Variável", "id": "variavel", "editable": False},
                    {"name": "Descrição", "id": "label",   "editable": False},
                    {"name": "Alpha (0.001 .. 1.0)", "id": "alpha",
                     "type": "numeric", "editable": True},
                ],
                data=ema_filter.all_as_table(),
                style_cell={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                            "border": f"1px solid {COL_BORDER}", "padding": "8px"},
                style_header={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                              "fontWeight": 600},
                style_data_conditional=[{
                    "if": {"column_editable": True},
                    "backgroundColor": COL_PANEL2,
                }],
            ),
            html.Div([
                html.Button("💾 Salvar filtros", id="ema-save", n_clicks=0,
                            style={"backgroundColor": COL_ACCENT, "color": COL_BG,
                                   "border": "none", "padding": "8px 14px",
                                   "borderRadius": "6px", "cursor": "pointer",
                                   "fontWeight": 600, "marginTop": "10px"}),
                html.Span(id="ema-save-feedback", style={"marginLeft": "12px",
                                                          "color": COL_MUTED}),
            ]),
        ], style=STYLE_PANEL),
    ])


def build_alerts_tab() -> html.Div:
    return html.Div([
        # Regras
        html.Div([
            html.H3("Regras de alerta", style={"marginTop": 0, "color": COL_TEXT}),
            html.Div([
                "Operadores válidos: ",
                html.Code(" > ", style={"color": COL_ACCENT}),
                html.Code(" < ", style={"color": COL_ACCENT}),
                html.Code(" >= ", style={"color": COL_ACCENT}),
                html.Code(" <= ", style={"color": COL_ACCENT}),
                html.Code(" == ", style={"color": COL_ACCENT}),
                ". Avaliação sobre valor FILTRADO (pós-EMA, pós-calibração). "
                "Borda de subida (não dispara repetidamente).",
            ], style={"color": COL_MUTED, "fontSize": "13px", "marginBottom": "10px"}),
            dash_table.DataTable(
                id="alerts-table",
                columns=[
                    {"name": "ID",        "id": "id",        "editable": False},
                    {"name": "Variável",  "id": "variable",
                     "presentation": "dropdown", "editable": True},
                    {"name": "Operador",  "id": "operator",
                     "presentation": "dropdown", "editable": True},
                    {"name": "Threshold", "id": "threshold", "type": "numeric", "editable": True},
                    {"name": "Mensagem",  "id": "message",   "editable": True},
                    {"name": "Ativo (sim/não)", "id": "enabled", "editable": True},
                ],
                data=alert_engine.all_as_table(),
                dropdown={
                    "variable": {"options": [{"label": k, "value": k} for k in VAR_KEYS]},
                    "operator": {"options": [{"label": o, "value": o} for o in ALERT_OPERATORS]},
                },
                editable=True,
                row_deletable=True,
                style_cell={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                            "border": f"1px solid {COL_BORDER}", "padding": "8px"},
                style_header={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                              "fontWeight": 600},
                css=[{"selector": ".Select-menu-outer", "rule": "color: #000;"}],
            ),
            html.Div([
                html.Button("➕ Nova regra", id="alerts-add", n_clicks=0,
                            style={"backgroundColor": COL_OK, "color": COL_BG,
                                   "border": "none", "padding": "8px 14px",
                                   "borderRadius": "6px", "cursor": "pointer",
                                   "fontWeight": 600, "marginTop": "10px",
                                   "marginRight": "8px"}),
                html.Button("💾 Salvar regras", id="alerts-save", n_clicks=0,
                            style={"backgroundColor": COL_ACCENT, "color": COL_BG,
                                   "border": "none", "padding": "8px 14px",
                                   "borderRadius": "6px", "cursor": "pointer",
                                   "fontWeight": 600, "marginTop": "10px",
                                   "marginRight": "8px"}),
                html.Button("🗑 Limpar log", id="alerts-clearlog", n_clicks=0,
                            style={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                                   "border": f"1px solid {COL_BORDER}",
                                   "padding": "8px 14px", "borderRadius": "6px",
                                   "cursor": "pointer", "fontWeight": 600,
                                   "marginTop": "10px"}),
                html.Span(id="alerts-save-feedback",
                          style={"marginLeft": "12px", "color": COL_MUTED}),
            ]),
        ], style=STYLE_PANEL),

        # Log de eventos
        html.Div([
            html.H3("Log de eventos (mais recentes no topo)",
                    style={"marginTop": 0, "color": COL_TEXT}),
            dash_table.DataTable(
                id="alerts-log",
                columns=[
                    {"name": "Timestamp", "id": "timestamp"},
                    {"name": "Variável",  "id": "variable"},
                    {"name": "Condição",  "id": "condicao"},
                    {"name": "Valor",     "id": "valor"},
                    {"name": "Mensagem",  "id": "mensagem"},
                ],
                data=[],
                page_size=15,
                style_cell={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                            "border": f"1px solid {COL_BORDER}", "padding": "6px 8px"},
                style_header={"backgroundColor": COL_PANEL2, "color": COL_TEXT,
                              "fontWeight": 600},
            ),
        ], style=STYLE_PANEL),
    ])


def build_status_bar() -> html.Div:
    return html.Div(id="status-bar", style={
        **STYLE_PANEL_TIGHT,
        "display": "flex", "gap": "12px", "alignItems": "center",
        "flexWrap": "wrap", "marginBottom": "10px",
    })


app.layout = html.Div(style=STYLE_PAGE, children=[
    html.Div([
        html.H2("🏎 Telemetria Baja SAE",
                style={"margin": "12px 0 8px 0", "color": COL_TEXT}),
        html.Div("Central STM32 → USART3 → ESP32-S3 → ESP-NOW/BajaLink → "
                 "ESP32-S3 gateway → USB CDC → Dashboard",
                 style={"color": COL_MUTED, "fontSize": "13px", "marginBottom": "8px"}),
    ]),
    build_status_bar(),

    dcc.Tabs(id="tabs", value="dashboard", colors={
        "border": COL_BORDER, "primary": COL_ACCENT, "background": COL_PANEL,
    }, children=[
        dcc.Tab(label="Dashboard", value="dashboard",
                style={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                       "border": f"1px solid {COL_BORDER}"},
                selected_style={"backgroundColor": COL_PANEL2, "color": COL_ACCENT,
                                "border": f"1px solid {COL_ACCENT}", "fontWeight": 600}),
        dcc.Tab(label="Configuração", value="config",
                style={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                       "border": f"1px solid {COL_BORDER}"},
                selected_style={"backgroundColor": COL_PANEL2, "color": COL_ACCENT,
                                "border": f"1px solid {COL_ACCENT}", "fontWeight": 600}),
        dcc.Tab(label="Alertas", value="alertas",
                style={"backgroundColor": COL_PANEL, "color": COL_TEXT,
                       "border": f"1px solid {COL_BORDER}"},
                selected_style={"backgroundColor": COL_PANEL2, "color": COL_ACCENT,
                                "border": f"1px solid {COL_ACCENT}", "fontWeight": 600}),
    ]),

    html.Div(id="tab-dashboard", className="tab-content", children=build_dashboard_tab()),
    html.Div(id="tab-config",    className="tab-content",
             style={"display": "none"}, children=build_config_tab()),
    html.Div(id="tab-alertas",   className="tab-content",
             style={"display": "none"}, children=build_alerts_tab()),

    # Intervals e stores
    dcc.Interval(id="tick", interval=UI_TICK_MS, n_intervals=0),
    dcc.Store(id="graph-reset-signal", data=0),
])


# =============================================================================
# SECTION 14 — CALLBACKS
# =============================================================================

# ---- Navegação entre abas (toggle de visibilidade, preserva estado) --------

@app.callback(
    Output("tab-dashboard", "style"),
    Output("tab-config",    "style"),
    Output("tab-alertas",   "style"),
    Input("tabs", "value"),
)
def cb_switch_tabs(tab: str) -> tuple[dict, dict, dict]:
    show = {"display": "block"}
    hide = {"display": "none"}
    return (
        show if tab == "dashboard" else hide,
        show if tab == "config"    else hide,
        show if tab == "alertas"   else hide,
    )


# ---- Visibilidade dos painéis por modo ------------------------------------

@app.callback(
    Output("playback-panel", "style"),
    Output("live-logging-panel", "style"),
    Input("src-radio", "value"),
)
def cb_panels_visibility(src: str) -> tuple[dict, dict]:
    if src == "playback":
        return STYLE_PANEL, {**STYLE_PANEL, "display": "none"}
    return {**STYLE_PANEL, "display": "none"}, STYLE_PANEL


# ---- Source switch (live <-> playback) -------------------------------------

@app.callback(
    Output("playback-status", "children"),
    Output("graph-reset-signal", "data", allow_duplicate=True),
    Input("src-radio", "value"),
    State("graph-reset-signal", "data"),
    prevent_initial_call=True,
)
def cb_source_switch(src: str, reset_gen: int) -> tuple[str, int]:
    gen = (reset_gen or 0) + 1
    if src == "live":
        if playback_mgr.is_running():
            playback_mgr.stop()
        data_store.reset_session()
        ema_filter.reset_state()
        graph_tracker.reset("main-graph")
        serial_mgr.start()
        return "Modo Live ativo. Aguardando receptor USB...", gen
    else:
        serial_mgr.stop()
        data_store.reset_session()
        ema_filter.reset_state()
        graph_tracker.reset("main-graph")
        return "Modo Playback ativo. Selecione ou informe o CSV e clique em Play.", gen


# ---- Controles de playback -------------------------------------------------

@app.callback(
    Output("playback-status", "children", allow_duplicate=True),
    Output("graph-reset-signal", "data", allow_duplicate=True),
    Input("pb-play",  "n_clicks"),
    Input("pb-pause", "n_clicks"),
    Input("pb-stop",  "n_clicks"),
    Input("pb-speed", "value"),
    State("playback-path", "value"),
    State("graph-reset-signal", "data"),
    prevent_initial_call=True,
)
def cb_playback_controls(_p: int, _pa: int, _s: int, speed: float,
                         path: Optional[str], reset_gen: int) -> Any:
    triggered = ctx.triggered_id
    gen = (reset_gen or 0) + 1
    if triggered == "pb-speed":
        playback_mgr.set_speed(float(speed or 1))
        return f"Velocidade ajustada para {speed}x.", no_update
    if triggered == "pb-play":
        if not path:
            return "⚠ Informe o caminho do arquivo CSV antes de tocar.", no_update
        p = Path(path).expanduser()
        if not p.is_absolute():
            p = BASE_DIR / p
        if not p.exists():
            return f"⚠ Arquivo não encontrado: {p}", no_update
        if playback_mgr.is_paused():
            playback_mgr.resume()
            return f"▶ Retomando: {p.name}", no_update
        # (re)start — reset completo do gráfico
        data_store.reset_session()
        ema_filter.reset_state()
        graph_tracker.reset("main-graph")
        ok = playback_mgr.start(p, float(speed or 1))
        msg = f"▶ Reproduzindo {p.name}" if ok else "⚠ Falha ao iniciar playback."
        return msg, gen
    if triggered == "pb-pause":
        if playback_mgr.is_running() and not playback_mgr.is_paused():
            playback_mgr.pause()
            return "⏸ Pausado.", no_update
        return no_update, no_update
    if triggered == "pb-stop":
        playback_mgr.stop()
        return "⏹ Parado.", no_update
    return no_update, no_update


# ---- Inicialização do gráfico principal (rebuild full) ---------------------

@app.callback(
    Output("main-graph", "figure"),
    Input("selected-vars", "value"),
    Input("graph-reset-signal", "data"),
)
def cb_init_main_graph(selected_vars: Optional[list[str]], _reset: int) -> go.Figure:
    selected = selected_vars or []
    graph_tracker.reset("main-graph")
    return make_empty_main_figure(selected)


# ---- Tick do gráfico principal (PATCH incremental) -------------------------

@app.callback(
    Output("main-graph", "figure", allow_duplicate=True),
    Input("tick", "n_intervals"),
    State("selected-vars", "value"),
    prevent_initial_call=True,
)
def cb_tick_main_graph(_n: int, selected_vars: Optional[list[str]]) -> Any:
    """
    Substitui os arrays x/y de cada trace diretamente via dash.Patch.
    Evita acúmulo de operações extend+del que dessincronizavam servidor/cliente.
    """
    selected = selected_vars or []
    if not selected:
        return no_update

    patched = Patch()
    any_update = False

    for i, var in enumerate(selected):
        prev_last_ts, _ = graph_tracker.get("main-graph", var)
        samples = data_store.get_recent_samples(var, MAX_GRAPH_POINTS)

        if not samples:
            # DataStore foi limpo; zera o trace se havia dados antes
            if prev_last_ts is not None:
                patched["data"][i]["x"] = []
                patched["data"][i]["y"] = []
                graph_tracker.set("main-graph", var, None, 0)
                any_update = True
            continue

        last_ts = samples[-1][0]
        if last_ts == prev_last_ts:
            continue  # Sem dados novos

        patched["data"][i]["x"] = [s[0] for s in samples]
        patched["data"][i]["y"] = [s[1] for s in samples]
        graph_tracker.set("main-graph", var, last_ts, len(samples))
        any_update = True

    if not any_update:
        return no_update
    return patched


# ---- Tiles numéricos -------------------------------------------------------

@app.callback(
    Output("tiles-container", "children"),
    Input("tick", "n_intervals"),
    State("selected-vars", "value"),
)
def cb_tiles(_n: int, selected_vars: Optional[list[str]]) -> list[html.Div]:
    selected = selected_vars or []
    stats = data_store.snapshot_stats()
    last_pkt = stats.get("last_packet") or {}
    active_vars = {r.variable for r in alert_engine.active_rules()}

    tiles: list[html.Div] = []
    for var in selected:
        meta = TELEMETRY_VARIABLES.get(var, {"label": var, "unit": ""})
        val = last_pkt.get(f"{var}_filt", last_pkt.get(var))
        if val is None:
            text_val = "—"
        else:
            try:
                text_val = f"{float(val):.2f}"
            except (TypeError, ValueError):
                text_val = str(val)
        is_alert = var in active_vars
        bg = COL_ERR if is_alert else COL_PANEL
        cls = "blink-alert" if is_alert else ""
        tiles.append(html.Div(className=cls, style={
            "backgroundColor": bg, "border": f"1px solid {COL_BORDER}",
            "borderRadius": "8px", "padding": "10px 12px",
        }, children=[
            html.Div(meta["label"], style=STYLE_LABEL),
            html.Div([
                html.Span(text_val, style=STYLE_VALUE),
                html.Span(f" {meta['unit']}",
                          style={"color": COL_MUTED, "fontSize": "12px", "marginLeft": "4px"}),
            ]),
        ]))
    return tiles


# ---- Barra de status -------------------------------------------------------

@app.callback(
    Output("status-bar", "children"),
    Input("tick", "n_intervals"),
    State("src-radio", "value"),
)
def cb_status_bar(_n: int, src: str) -> list[Any]:
    stats = data_store.snapshot_stats()
    last = stats.get("last_packet") or {}

    if src == "live":
        connected = serial_mgr.is_connected()
        port = serial_mgr.port_name() or "—"
        if connected:
            kind = serial_mgr.device_kind()
            conn_pill = make_pill(
                f"● CONECTADO {port}" + (f" · {kind}" if kind else ""), COL_OK)
        else:
            conn_pill = make_pill("○ AGUARDANDO USB", COL_WARN)
        csv_pill = make_pill(
            f"📄 {serial_mgr.csv_path().name}" if serial_mgr.csv_path() else "📄 —",
            COL_PANEL2, fg=COL_TEXT,
        )
    else:
        running = playback_mgr.is_running()
        paused = playback_mgr.is_paused()
        if running and not paused:
            conn_pill = make_pill("▶ PLAYBACK", COL_ACCENT)
        elif paused:
            conn_pill = make_pill("⏸ PAUSADO", COL_WARN)
        else:
            conn_pill = make_pill("⏹ STOPPED", COL_PANEL2,
                                  fg=COL_TEXT,
                                  extra_style={"border": f"1px solid {COL_BORDER}"})
        prog = playback_mgr.progress() * 100.0
        csv_pill = make_pill(
            f"📄 {playback_mgr.csv_path().name} ({prog:.0f}%)"
            if playback_mgr.csv_path() else "📄 —",
            COL_PANEL2, fg=COL_TEXT,
        )

    loss_pct = stats.get("loss_pct", 0.0)
    loss_pill = make_pill(f"Perda: {loss_pct:.2f}%", loss_color(loss_pct))
    rx_pill = make_pill(f"RX: {stats.get('total_rx', 0)}",
                        COL_PANEL2, fg=COL_TEXT,
                        extra_style={"border": f"1px solid {COL_BORDER}"})
    rst_pill = make_pill(f"Resets: {stats.get('resets', 0)}",
                         COL_PANEL2, fg=COL_TEXT,
                         extra_style={"border": f"1px solid {COL_BORDER}"})

    # E-STOP
    estop_raw = last.get("estop", 0)
    try:
        estop_on = int(estop_raw) != 0
    except (TypeError, ValueError):
        estop_on = False
    if estop_on:
        estop_pill = html.Span("⚠ E-STOP ATIVO", className="blink-alert",
                               style={**STYLE_PILL_BASE,
                                      "backgroundColor": COL_ERR, "color": COL_TEXT})
    else:
        estop_pill = make_pill("E-STOP OK", COL_OK)

    # Alertas ativos
    n_active = alert_engine.active_count()
    if n_active > 0:
        alert_pill = html.Span(f"🔔 {n_active} alerta(s) ativo(s)",
                               className="blink-alert",
                               style={**STYLE_PILL_BASE,
                                      "backgroundColor": COL_ERR, "color": COL_TEXT})
    else:
        alert_pill = make_pill("Sem alertas", COL_OK)

    # Saude do link. Com o gateway ESP-NOW usa o objeto "link"; sem ele,
    # cai no "tele" do receptor LoRa, que e o que os logs antigos tem.
    link = last.get("link") or {}
    neutral = {"border": f"1px solid {COL_BORDER}"}

    def li(k: str) -> int:
        try:
            return int(float(link.get(k, 0) or 0))
        except (TypeError, ValueError):
            return 0

    if link:
        pdr = 0.0
        try:
            pdr = float(link.get("pdr", 0.0) or 0.0)
        except (TypeError, ValueError):
            pass
        buf, bmax = li("buf"), li("bmax")
        bpct = (100.0 * buf / bmax) if bmax else 0.0
        link_pills = [
            make_pill("ESP-NOW associado" if li("up") else "ESP-NOW procurando",
                      COL_OK if li("up") else COL_WARN),
            make_pill(f"PDR {pdr:.1f}%", loss_color(100.0 - pdr)),
            make_pill(f"RSSI {li('rssi')} dBm", rssi_color(li("rssi"))),
            make_pill(f"Buffer {buf}/{bmax}",
                      COL_OK if bpct < 25 else (COL_WARN if bpct < 70 else COL_ERR)),
            make_pill(f"retx {li('retx')} · lacunas {li('gap')}",
                      COL_PANEL2, fg=COL_TEXT, extra_style=neutral),
            make_pill(f"RTT {li('srtt')} ms · cwnd {li('cwnd')} · "
                      f"ch {li('ch')}" + (" · LR" if li("lr") else ""),
                      COL_PANEL2, fg=COL_TEXT, extra_style=neutral),
        ]
    else:
        tele = last.get("tele") or {}
        link_pills = [make_pill(
            f"LoRa: rx={tele.get('rx', '—')} lost={tele.get('lost', '—')} "
            f"loss={tele.get('loss_pct', '—')}% rst={tele.get('rst', '—')}",
            COL_PANEL2, fg=COL_TEXT, extra_style=neutral)]

    return [conn_pill, csv_pill, rx_pill, loss_pill, rst_pill,
            estop_pill, alert_pill] + link_pills


# ---- Painel do link ESP-NOW ------------------------------------------------

@app.callback(
    Output("link-graph", "figure"),
    Output("link-tiles", "children"),
    Input("tick", "n_intervals"),
)
def cb_link_panel(_n: int) -> tuple[go.Figure, list[Any]]:
    stats = data_store.snapshot_stats()
    last = stats.get("last_packet") or {}
    link = last.get("link") or {}

    # As series do link sao achatadas no pacote em _process_line, entao saem
    # do mesmo buffer das variaveis de telemetria. use_filtered=False de
    # proposito: EMA em contador e indicador de link nao faz sentido.
    rssi = data_store.get_recent_samples("link_rssi", MAX_GRAPH_POINTS, use_filtered=False)
    pdr = data_store.get_recent_samples("link_pdr", MAX_GRAPH_POINTS, use_filtered=False)
    bufs = data_store.get_recent_samples("link_buf", MAX_GRAPH_POINTS, use_filtered=False)
    bmax = data_store.get_recent_samples("link_bmax", MAX_GRAPH_POINTS, use_filtered=False)

    # Ocupacao em porcentagem. As duas series vem dos mesmos pacotes, na mesma
    # ordem, o que permite emparelhar por indice.
    buf_pct: list[tuple[float, float]] = []
    for (ts, used), (_, total) in zip(bufs, bmax):
        if total and total > 0:
            buf_pct.append((ts, 100.0 * used / total))

    return build_link_figure(rssi, pdr, buf_pct), build_link_tiles(link)


# ---- Atitude 3D ------------------------------------------------------------

@app.callback(
    Output("attitude-graph", "figure"),
    Output("attitude-pitch-val", "children"),
    Output("attitude-roll-val", "children"),
    Input("tick", "n_intervals"),
)
def cb_attitude(_n: int) -> tuple[go.Figure, str, str]:
    stats = data_store.snapshot_stats()
    pitch = float(stats.get("last_pitch", 0.0) or 0.0)
    roll = float(stats.get("last_roll", 0.0) or 0.0)
    return (
        build_attitude_figure(pitch, roll),
        f"{pitch:+.1f}°",
        f"{roll:+.1f}°",
    )


# ---- Calibração: salvar ----------------------------------------------------

@app.callback(
    Output("cal-save-feedback", "children"),
    Input("cal-save", "n_clicks"),
    State("cal-table", "data"),
    prevent_initial_call=True,
)
def cb_save_cal(_n: int, rows: list[dict[str, Any]]) -> str:
    calibration_mgr.update_from_table(rows or [])
    return f"✓ Salvo em {datetime.now().strftime('%H:%M:%S')}"


# ---- EMA: salvar -----------------------------------------------------------

@app.callback(
    Output("ema-save-feedback", "children"),
    Input("ema-save", "n_clicks"),
    State("ema-table", "data"),
    prevent_initial_call=True,
)
def cb_save_ema(_n: int, rows: list[dict[str, Any]]) -> str:
    ema_filter.update_from_table(rows or [])
    return f"✓ Salvo em {datetime.now().strftime('%H:%M:%S')}"


# ---- Alertas: adicionar/salvar/limpar log ----------------------------------

@app.callback(
    Output("alerts-table", "data"),
    Output("alerts-save-feedback", "children"),
    Input("alerts-add",      "n_clicks"),
    Input("alerts-save",     "n_clicks"),
    Input("alerts-clearlog", "n_clicks"),
    State("alerts-table", "data"),
    prevent_initial_call=True,
)
def cb_alerts_actions(_a: int, _s: int, _c: int,
                       rows: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], str]:
    triggered = ctx.triggered_id
    rows = rows or []
    if triggered == "alerts-add":
        rows = list(rows) + [{
            "id": uuid.uuid4().hex[:8],
            "variable": VAR_KEYS[0],
            "operator": ">",
            "threshold": 0.0,
            "message": "Nova regra",
            "enabled": "sim",
        }]
        return rows, "Nova linha adicionada — clique em Salvar para persistir."
    if triggered == "alerts-save":
        alert_engine.update_from_table(rows)
        return alert_engine.all_as_table(), f"✓ Salvo em {datetime.now().strftime('%H:%M:%S')}"
    if triggered == "alerts-clearlog":
        alert_engine.clear_log()
        return no_update, f"Log limpo em {datetime.now().strftime('%H:%M:%S')}"
    return no_update, no_update


# ---- Alertas: log table (tick) ---------------------------------------------

@app.callback(
    Output("alerts-log", "data"),
    Input("tick", "n_intervals"),
)
def cb_alerts_log(_n: int) -> list[dict[str, Any]]:
    return alert_engine.events_as_table()


# ---- Controle de logging live ----------------------------------------------

@app.callback(
    Output("log-status", "children"),
    Input("log-start", "n_clicks"),
    Input("log-pause", "n_clicks"),
    Input("log-stop",  "n_clicks"),
    prevent_initial_call=True,
)
def cb_logging_controls(_s: int, _p: int, _st: int) -> Any:
    triggered = ctx.triggered_id
    if triggered == "log-start":
        if serial_mgr.is_logging_paused():
            serial_mgr.resume_logging()
            name = serial_mgr.csv_path().name if serial_mgr.csv_path() else "—"
            return f"⏺ Gravando: {name}"
        name = serial_mgr.start_logging()
        return f"⏺ Gravando: {name}" if name else "⏺ Aguardando conexão para iniciar log..."
    if triggered == "log-pause":
        serial_mgr.pause_logging()
        return "⏸ Log pausado."
    if triggered == "log-stop":
        serial_mgr.stop_logging()
        return "⏹ Log parado."
    return no_update


# ---- Lista de logs disponíveis (dropdown playback) -------------------------

@app.callback(
    Output("pb-log-dropdown", "options"),
    Input("pb-refresh-logs", "n_clicks"),
    Input("src-radio", "value"),
)
def cb_refresh_log_list(_n: int, src: str) -> list[dict[str, str]]:
    if not CSV_DIR.exists():
        return []
    files = sorted(CSV_DIR.glob("*.csv"), key=lambda f: f.stat().st_mtime, reverse=True)
    return [{"label": f.name, "value": str(f)} for f in files]


@app.callback(
    Output("playback-path", "value"),
    Input("pb-log-dropdown", "value"),
    prevent_initial_call=True,
)
def cb_log_dropdown_to_path(selected: Optional[str]) -> Any:
    if not selected:
        return no_update
    return selected


# =============================================================================
# SECTION 15 — ENTRY POINT
# =============================================================================

def main() -> None:
    log.info("=" * 60)
    log.info("Dashboard de Telemetria Baja SAE v2.0")
    log.info("=" * 60)
    log.info("Base dir:  %s", BASE_DIR)
    log.info("CSV dir:   %s", CSV_DIR)
    log.info("Calibração: %s", CALIBRATION_FILE)
    log.info("Filtros:    %s", FILTERS_FILE)
    log.info("Alertas:    %s", ALERTS_FILE)
    log.info("Dashboard em http://127.0.0.1:8050")
    log.info("=" * 60)
    # debug=False intencional: o reloader do Flask cria 2 processos e duplica
    # a thread serial, gerando conflito na porta CDC.
    app.run(debug=False, host="127.0.0.1", port=8050)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log.info("Interrompido pelo usuário. Encerrando...")
    finally:
        serial_mgr.stop()
        playback_mgr.stop()
        log.info("Encerrado.")
