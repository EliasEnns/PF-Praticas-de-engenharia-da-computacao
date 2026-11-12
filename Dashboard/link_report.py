#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Relatório de desempenho do link a partir dos logs CSV.
========================================================

Transforma um log gravado pelo dashboard num relatório em Markdown com os
números medidos. A intenção é que nenhum valor do relatório final do projeto
seja estimado de cabeça: tudo que se afirma sobre taxa, latência, perda e
alcance sai daqui, de um arquivo que existe no disco.

Lê tanto os logs novos, que têm as colunas link_*, quanto os antigos do
caminho LoRa, que não têm. Isso permite a comparação que é o ponto central da
proposta do projeto: quantos records por segundo o caminho LoRa entregava e
quantos o caminho ESP-NOW entrega.

Uso:
    # relatório de uma sessão
    python tools/link_report.py "Dash teste Claude/logs/telemetria_AAAAMMDD_HHMMSS.csv"

    # comparando com a linha de base do LoRa
    python tools/link_report.py novo.csv --baseline "logs/telemetria_20260515_155311.csv"

    # gravando em arquivo
    python tools/link_report.py novo.csv -o docs/RESULTADOS.md
"""

from __future__ import annotations

import argparse
import csv
import re
import statistics as st
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Optional


# ---------------------------------------------------------------------------
# Leitura
# ---------------------------------------------------------------------------

def ler(caminho: Path) -> list[dict[str, str]]:
    with caminho.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def num(linhas: list[dict[str, str]], col: str) -> list[float]:
    """Coluna como floats, descartando vazios e não numéricos."""
    out: list[float] = []
    for r in linhas:
        v = r.get(col, "")
        if v is None or v == "":
            continue
        try:
            out.append(float(v))
        except ValueError:
            continue
    return out


def ultimo(linhas: list[dict[str, str]], col: str) -> Optional[float]:
    vals = num(linhas, col)
    return vals[-1] if vals else None


def pico(linhas: list[dict[str, str]], col: str) -> Optional[float]:
    vals = num(linhas, col)
    return max(vals) if vals else None


def pct(vals: list[float], q: float) -> float:
    """Percentil por interpolação linear, sem depender de numpy."""
    if not vals:
        return float("nan")
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    i = (len(s) - 1) * q
    lo, hi = int(i), min(int(i) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (i - lo)


# ---------------------------------------------------------------------------
# Métricas
# ---------------------------------------------------------------------------

def intervalos_ms(linhas: list[dict[str, str]]) -> list[float]:
    """Intervalos entre records consecutivos, pelo ts do MCU.

    O ts do PC não serve aqui: ele mede quando o Python conseguiu ler a linha,
    o que embute o agendamento do sistema operacional e o buffer do driver USB.
    O ts do MCU é gerado na origem do dado.
    """
    ts = num(linhas, "ts")
    return [b - a for a, b in zip(ts, ts[1:]) if 0 < (b - a) < 60_000]


def analisar(linhas: list[dict[str, str]], nome: str) -> dict[str, Any]:
    d: dict[str, Any] = {"nome": nome, "n": len(linhas)}
    if not linhas:
        return d

    tem_link = any(k.startswith("link_") for k in linhas[0].keys())
    d["tem_link"] = tem_link

    # Duração pela referência do MCU, com o relógio do PC como conferência.
    ts = num(linhas, "ts")
    if len(ts) >= 2:
        d["dur_s"] = (ts[-1] - ts[0]) / 1000.0
    else:
        d["dur_s"] = 0.0

    pc = []
    for r in linhas:
        v = r.get("pc_ts", "")
        if v:
            try:
                pc.append(datetime.fromisoformat(v))
            except ValueError:
                pass
    if len(pc) >= 2:
        d["dur_pc_s"] = (pc[-1] - pc[0]).total_seconds()
        d["inicio"] = pc[0].isoformat(timespec="seconds")

    if d["dur_s"] > 0:
        d["taxa_hz"] = (len(linhas) - 1) / d["dur_s"]

    iv = intervalos_ms(linhas)
    if iv:
        d["iv_med"] = st.fmean(iv)
        d["iv_p50"] = pct(iv, 0.50)
        d["iv_p95"] = pct(iv, 0.95)
        d["iv_p99"] = pct(iv, 0.99)
        d["iv_max"] = max(iv)
        # Jitter como desvio padrão do intervalo: é o que estraga análise de
        # frequência, mais do que a taxa média.
        d["iv_sd"] = st.pstdev(iv) if len(iv) > 1 else 0.0
        # Buracos: intervalo muito acima do nominal indica lacuna no stream.
        nominal = d.get("iv_p50", 0.0)
        if nominal > 0:
            buracos = [x for x in iv if x > max(3 * nominal, nominal + 50)]
            d["buracos_n"] = len(buracos)
            d["buracos_ms_total"] = sum(buracos)
            d["buracos_ms_maior"] = max(buracos) if buracos else 0.0

    # Sequência: perda medida de forma independente dos contadores do firmware.
    seq = [int(x) for x in num(linhas, "seq")]
    if len(seq) >= 2:
        perdidos = 0
        resets = 0
        for a, b in zip(seq, seq[1:]):
            dif = b - a
            if dif == 1:
                continue
            if dif > 1:
                perdidos += dif - 1
            else:
                resets += 1
        d["seq_perdidos"] = perdidos
        d["seq_resets"] = resets
        esperados = perdidos + len(seq)
        d["seq_entrega_pct"] = 100.0 * len(seq) / esperados if esperados else 100.0

    if not tem_link:
        return d

    # --- métricas do link ESP-NOW ---
    rssi = num(linhas, "link_rssi")
    if rssi:
        d["rssi_med"] = st.fmean(rssi)
        d["rssi_p05"] = pct(rssi, 0.05)
        d["rssi_min"] = min(rssi)
        d["rssi_max"] = max(rssi)

    pdr = num(linhas, "link_pdr")
    if pdr:
        d["pdr_med"] = st.fmean(pdr)
        d["pdr_p05"] = pct(pdr, 0.05)
        d["pdr_min"] = min(pdr)

    srtt = [x for x in num(linhas, "link_srtt") if x > 0]
    if srtt:
        d["rtt_med"] = st.fmean(srtt)
        d["rtt_p95"] = pct(srtt, 0.95)
        d["rtt_max"] = max(srtt)

    cwnd = num(linhas, "link_cwnd")
    if cwnd:
        d["cwnd_med"] = st.fmean(cwnd)
        d["cwnd_min"] = min(cwnd)
        d["cwnd_max"] = max(cwnd)

    buf = num(linhas, "link_buf")
    bmax = ultimo(linhas, "link_bmax") or 0.0
    if buf:
        d["buf_pico"] = max(buf)
        d["buf_total"] = bmax
        if bmax > 0:
            d["buf_pico_pct"] = 100.0 * max(buf) / bmax
            # Autonomia de queda que o buffer sustenta na taxa medida.
            if d.get("taxa_hz"):
                d["buf_autonomia_s"] = bmax / d["taxa_hz"]

    # Contadores cumulativos: o valor final é o total da sessão, desde que o
    # gateway não tenha sido reiniciado no meio.
    for k, col in (("frames", "link_fr"), ("retx", "link_retx"),
                   ("lacunas", "link_gap"), ("dups", "link_dup"),
                   ("estouro", "link_drp"), ("abandonados", "link_gu"),
                   ("quedas", "link_ld")):
        v = pico(linhas, col)
        if v is not None:
            d[k] = v

    if d.get("frames") and d.get("retx") is not None:
        enviados = d["frames"] + d["retx"]
        d["retx_pct"] = 100.0 * d["retx"] / enviados if enviados else 0.0
    if d.get("frames") is not None and d.get("lacunas") is not None:
        tot = d["frames"] + d["lacunas"]
        d["pdr_sessao"] = 100.0 * d["frames"] / tot if tot else 100.0

    ch = ultimo(linhas, "link_ch")
    lr = ultimo(linhas, "link_lr")
    src = ultimo(linhas, "link_src")
    hz = ultimo(linhas, "link_hz")
    d["canal"] = int(ch) if ch is not None else None
    d["lr"] = int(lr) if lr is not None else None
    d["fonte"] = ("UART" if src else "emulador") if src is not None else None
    d["fonte_hz"] = int(hz) if hz is not None else None
    return d


# ---------------------------------------------------------------------------
# Modo alcance: segmentação do log por janela de tempo
# ---------------------------------------------------------------------------

def segmentar(linhas: list[dict[str, str]], janela_s: float) -> list[dict[str, Any]]:
    """Divide a sessão em janelas de tempo e resume cada uma.

    Para o ensaio de alcance, o procedimento é caminhar parando a cada marca.
    Cada parada vira uma janela com RSSI estável; os trechos de caminhada viram
    janelas com RSSI variando. O desvio padrão do RSSI dentro da janela separa
    uma coisa da outra sem precisar anotar horário nenhum em campo: basta
    caminhar, parar, e depois casar as janelas estáveis com as marcas na ordem.
    """
    ts = num(linhas, "ts")
    if len(ts) < 2:
        return []
    t0 = ts[0]
    jan_ms = janela_s * 1000.0

    # Agrupa as linhas por índice de janela.
    grupos: dict[int, list[dict[str, str]]] = {}
    for r in linhas:
        v = r.get("ts", "")
        if not v:
            continue
        try:
            k = int((float(v) - t0) // jan_ms)
        except ValueError:
            continue
        grupos.setdefault(k, []).append(r)

    out: list[dict[str, Any]] = []
    for k in sorted(grupos):
        g = grupos[k]
        s: dict[str, Any] = {
            "idx": k,
            "t_ini": k * janela_s,
            "t_fim": (k + 1) * janela_s,
            "n": len(g),
        }
        rssi = num(g, "link_rssi")
        if rssi:
            s["rssi_med"] = st.fmean(rssi)
            s["rssi_sd"] = st.pstdev(rssi) if len(rssi) > 1 else 0.0
            s["rssi_min"] = min(rssi)
        pdr = num(g, "link_pdr")
        if pdr:
            s["pdr_med"] = st.fmean(pdr)
            s["pdr_min"] = min(pdr)

        # Contadores cumulativos: o que interessa é o quanto andaram na janela.
        for nome, col in (("retx", "link_retx"), ("lacunas", "link_gap"),
                          ("dups", "link_dup"), ("estouro", "link_drp")):
            v = num(g, col)
            if v:
                s[nome] = v[-1] - v[0]

        # Fração entregue: quantos records chegaram contra quantos a fonte
        # deveria ter produzido na janela. É o indicador mais direto de borda
        # de alcance, porque cai antes do PDR instantâneo chegar a zero.
        hz = num(g, "link_hz")
        if hz:
            esperado = st.fmean(hz) * janela_s
            if esperado > 0:
                s["entrega_pct"] = 100.0 * len(g) / esperado
                s["hz"] = st.fmean(hz)

        # Classificação: parada tem RSSI estável, caminhada não. O limiar de
        # 3 dB é folgado o suficiente para o desvanecimento normal de uma
        # pessoa parada segurando a placa.
        sd = s.get("rssi_sd")
        s["estavel"] = (sd is not None and sd < 3.0)
        out.append(s)

    # A ultima janela quase sempre esta cortada no meio, porque a sessao
    # terminou antes de completa-la. Sem marcar isso, ela aparece como queda
    # brusca de entrega e seria lida como borda de alcance, o que e falso.
    if out:
        out[-1]["parcial"] = True
    return out


def render_alcance(d: dict[str, Any], segs: list[dict[str, Any]],
                   janela_s: float) -> str:
    L: list[str] = []
    L.append("# Ensaio de alcance — segmentação do log")
    L.append("")
    L.append(f"Gerado por `tools/link_report.py --alcance` a partir de "
             f"`{d['nome']}`, em janelas de {janela_s:g} s.")
    L.append("")
    L.append("Como usar esta tabela: as linhas marcadas **parada** têm RSSI "
             "estável e correspondem aos pontos em que você parou. Preencha a "
             "coluna de distância nelas, na ordem da caminhada, e descarte as "
             "linhas de trânsito. O resultado é a tabela do ensaio T6.")
    L.append("")
    L.append("| # | t (s) | Tipo | Dist. | Records | Entrega | RSSI méd | RSSI σ | RSSI mín | PDR méd | PDR mín | Retx | Lacunas |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for s in segs:
        L.append("| {idx} | {t0:.0f}–{t1:.0f} | {tipo} |  | {n} | {ent} | "
                 "{rm} | {rsd} | {rmin} | {pm} | {pmin} | {retx} | {gap} |".format(
            idx=s["idx"],
            t0=s["t_ini"], t1=s["t_fim"],
            tipo=("parcial" if s.get("parcial")
                  else ("**parada**" if s.get("estavel") else "trânsito")),
            n=s["n"],
            ent=f(s.get("entrega_pct"), 1, " %"),
            rm=f(s.get("rssi_med"), 1),
            rsd=f(s.get("rssi_sd"), 1),
            rmin=f(s.get("rssi_min"), 0),
            pm=f(s.get("pdr_med"), 1),
            pmin=f(s.get("pdr_min"), 1),
            retx=f(s.get("retx"), 0),
            gap=f(s.get("lacunas"), 0),
        ))
    L.append("")

    paradas = [s for s in segs if s.get("estavel") and not s.get("parcial")]
    L.append(f"{len(segs)} janelas, {len(paradas)} classificadas como parada. "
             f"A última está marcada como parcial e foi ignorada na análise, "
             f"porque a sessão terminou antes de completá-la.")
    L.append("")

    # Borda de alcance: primeira parada em que a entrega cai de forma clara.
    borda = None
    for s in paradas:
        e = s.get("entrega_pct")
        if e is not None and e < 95.0:
            borda = s
            break
    if borda:
        L.append(f"Primeira parada com entrega abaixo de 95 %: janela "
                 f"{borda['idx']} ({borda['t_ini']:.0f}–{borda['t_fim']:.0f} s), "
                 f"RSSI médio {f(borda.get('rssi_med'), 1)} dBm, entrega "
                 f"{f(borda.get('entrega_pct'), 1)} %. É a marca em que o link "
                 f"começou a ficar sem folga.")
    else:
        L.append("Nenhuma parada ficou abaixo de 95 % de entrega: o alcance "
                 "útil não foi alcançado neste ensaio. Vale repetir indo mais "
                 "longe, ou com obstrução.")
    L.append("")
    L.append("Referência de leitura: acima de −75 dBm o link tem folga; entre "
             "−75 e −90 funciona com retransmissão aparecendo; abaixo de −90 "
             "está no limite da sensibilidade.")
    return "\n".join(L)


# ---------------------------------------------------------------------------
# Modo tabela: um log por marca, rótulo no nome do arquivo
# ---------------------------------------------------------------------------

def rotulo_do_nome(caminho: Path) -> str:
    """Extrai o rótulo de telemetria_AAAAMMDD_HHMMSS_<rotulo>.csv.

    O dashboard grava o rótulo digitado no nome do arquivo, então o ensaio de
    alcance não precisa de anotação em papel: o próprio diretório de logs já é
    a tabela, e só falta ordenar.
    """
    partes = caminho.stem.split("_")
    return "_".join(partes[3:]) if len(partes) >= 4 else caminho.stem


def ordem_rotulo(rot: str) -> tuple[float, str]:
    """Ordena por número no rótulo (0m, 25m, 100m), com o resto em ordem alfabética."""
    m = re.search(r"-?\d+(?:[.,]\d+)?", rot)
    return (float(m.group().replace(",", ".")) if m else float("inf"), rot)


def render_tabela(itens: list[tuple[str, dict[str, Any]]]) -> str:
    L: list[str] = []
    L.append("# Ensaio de alcance — tabela por marca")
    L.append("")
    L.append("Gerado por `tools/link_report.py --tabela`. Uma linha por log, o "
             "rótulo vindo do nome do arquivo. Todos os valores são medidos.")
    L.append("")
    L.append("| Marca | Records | Duração | Taxa | RSSI méd | RSSI mín | PDR sessão | Retx | Lacunas | Perda |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    for rot, d in itens:
        perda = (d.get("estouro") or 0) + (d.get("abandonados") or 0)
        L.append("| **{rot}** | {n} | {dur} | {hz} | {rm} | {rmin} | {pdr} | {retx} | {gap} | {perda} |".format(
            rot=rot, n=f(d.get("n"), 0), dur=f(d.get("dur_s"), 1, " s"),
            hz=f(d.get("taxa_hz"), 1, " Hz"),
            rm=f(d.get("rssi_med"), 1, " dBm"), rmin=f(d.get("rssi_min"), 0, " dBm"),
            pdr=f(d.get("pdr_sessao"), 2, " %"), retx=f(d.get("retx"), 0),
            gap=f(d.get("lacunas"), 0), perda=f(perda, 0)))
    L.append("")

    # Borda de alcance: primeira marca cuja entrega sai de 100 %.
    borda = None
    for rot, d in itens:
        pdr = d.get("pdr_sessao")
        if pdr is not None and pdr < 99.5:
            borda = (rot, d)
            break
    if borda:
        rot, d = borda
        L.append(f"Primeira marca com PDR abaixo de 99,5 %: **{rot}**, "
                 f"RSSI médio {f(d.get('rssi_med'), 1)} dBm. "
                 f"É onde o link deixou de ter folga.")
    else:
        L.append("Nenhuma marca ficou abaixo de 99,5 % de PDR: o alcance útil "
                 "não foi alcançado neste ensaio. Vale repetir indo mais longe, "
                 "ou com obstrução entre as placas.")
    L.append("")
    L.append("Referência de leitura: acima de −75 dBm o link tem folga; entre "
             "−75 e −90 funciona com retransmissão aparecendo; abaixo de −90 "
             "está no limite da sensibilidade.")
    return "\n".join(L)


# ---------------------------------------------------------------------------
# Saída
# ---------------------------------------------------------------------------

def f(v: Any, casas: int = 2, suf: str = "") -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        if v != v:                       # NaN
            return "—"
        return f"{v:.{casas}f}{suf}"
    return f"{v}{suf}"


def tabela(linhas: list[tuple[str, str]]) -> str:
    out = ["| Métrica | Valor |", "|---|---|"]
    out += [f"| {a} | {b} |" for a, b in linhas]
    return "\n".join(out)


def render(d: dict[str, Any], base: Optional[dict[str, Any]]) -> str:
    L: list[str] = []
    L.append("# Resultados medidos — link BajaLink / ESP-NOW")
    L.append("")
    L.append(f"Gerado por `tools/link_report.py` a partir de `{d['nome']}`.")
    L.append("Todos os valores são medidos; nenhum é estimado.")
    L.append("")

    L.append("## Sessão")
    L.append("")
    L.append(tabela([
        ("Início", f(d.get("inicio"))),
        ("Records no log", f(d.get("n"), 0)),
        ("Duração (ts do MCU)", f(d.get("dur_s"), 1, " s")),
        ("Taxa média de entrega", f(d.get("taxa_hz"), 2, " records/s")),
        ("Fonte de dados", f(d.get("fonte"))),
        ("Taxa nominal da fonte", f(d.get("fonte_hz"), 0, " Hz")),
        ("Canal", f(d.get("canal"), 0)),
        ("Modo long range", {0: "desligado", 1: "LR + legado",
                             2: "LR exclusivo"}.get(d.get("lr"), "—")),
    ]))
    L.append("")

    L.append("## Cadência e jitter")
    L.append("")
    L.append("O intervalo entre records é medido pelo `ts` do MCU, não pelo "
             "relógio do PC, para não embutir o agendamento do sistema "
             "operacional nem o buffer do driver USB.")
    L.append("")
    L.append(tabela([
        ("Intervalo médio", f(d.get("iv_med"), 2, " ms")),
        ("Mediana (p50)", f(d.get("iv_p50"), 2, " ms")),
        ("p95", f(d.get("iv_p95"), 2, " ms")),
        ("p99", f(d.get("iv_p99"), 2, " ms")),
        ("Máximo", f(d.get("iv_max"), 2, " ms")),
        ("Desvio padrão (jitter)", f(d.get("iv_sd"), 2, " ms")),
        ("Buracos no stream", f(d.get("buracos_n"), 0)),
        ("Tempo total em buraco", f(d.get("buracos_ms_total"), 0, " ms")),
        ("Maior buraco", f(d.get("buracos_ms_maior"), 0, " ms")),
    ]))
    L.append("")

    L.append("## Integridade da sequência")
    L.append("")
    L.append("Contagem independente dos contadores do firmware: feita sobre o "
             "campo `seq` do próprio log, o que serve de conferência cruzada.")
    L.append("")
    L.append(tabela([
        ("Records perdidos (lacuna em seq)", f(d.get("seq_perdidos"), 0)),
        ("Resets de sequência", f(d.get("seq_resets"), 0)),
        ("Entrega medida por seq", f(d.get("seq_entrega_pct"), 3, " %")),
    ]))
    L.append("")

    if d.get("tem_link"):
        L.append("## Rádio")
        L.append("")
        L.append(tabela([
            ("RSSI médio", f(d.get("rssi_med"), 1, " dBm")),
            ("RSSI p05 (pior 5 %)", f(d.get("rssi_p05"), 1, " dBm")),
            ("RSSI mínimo", f(d.get("rssi_min"), 1, " dBm")),
            ("RSSI máximo", f(d.get("rssi_max"), 1, " dBm")),
        ]))
        L.append("")

        L.append("## Confiabilidade do link")
        L.append("")
        L.append(tabela([
            ("Quadros entregues", f(d.get("frames"), 0)),
            ("Retransmissões", f(d.get("retx"), 0)),
            ("Taxa de retransmissão", f(d.get("retx_pct"), 2, " %")),
            ("Duplicatas descartadas", f(d.get("dups"), 0)),
            ("Lacunas declaradas", f(d.get("lacunas"), 0)),
            ("PDR da sessão", f(d.get("pdr_sessao"), 3, " %")),
            ("PDR médio (janela 1 s)", f(d.get("pdr_med"), 2, " %")),
            ("PDR p05 (pior 5 %)", f(d.get("pdr_p05"), 2, " %")),
            ("Quedas de link", f(d.get("quedas"), 0)),
            ("Perda por estouro de buffer", f(d.get("estouro"), 0, " records")),
            ("Quadros abandonados por prazo", f(d.get("abandonados"), 0)),
        ]))
        L.append("")

        L.append("## Controle de fluxo e store-and-forward")
        L.append("")
        L.append(tabela([
            ("RTT estimado médio", f(d.get("rtt_med"), 1, " ms")),
            ("RTT p95", f(d.get("rtt_p95"), 1, " ms")),
            ("RTT máximo", f(d.get("rtt_max"), 1, " ms")),
            ("Janela média", f(d.get("cwnd_med"), 1, " quadros")),
            ("Janela mínima / máxima",
             f"{f(d.get('cwnd_min'), 0)} / {f(d.get('cwnd_max'), 0)}"),
            ("Buffer total do sensor", f(d.get("buf_total"), 0, " quadros")),
            ("Ocupação de pico do buffer", f(d.get("buf_pico"), 0, " quadros")),
            ("Ocupação de pico", f(d.get("buf_pico_pct"), 1, " %")),
            ("Autonomia de queda na taxa medida",
             f(d.get("buf_autonomia_s"), 1, " s")),
        ]))
        L.append("")
        if d.get("buf_pico_pct") is not None:
            if d["buf_pico_pct"] < 5:
                L.append("> O buffer praticamente não foi exercitado nesta "
                         "sessão: o link deu conta da taxa de aquisição em "
                         "tempo real. Para medir o store-and-forward é preciso "
                         "provocar a queda — ver o ensaio T5 do plano de "
                         "testes.")
            elif d["buf_pico_pct"] > 95:
                L.append("> O buffer encostou na capacidade. A queda durou "
                         "mais que a autonomia disponível e houve perda "
                         "definitiva. Ou se aumenta o buffer (PSRAM), ou se "
                         "aceita a perda e ela fica registrada — que é o que "
                         "a coluna de estouro mostra.")
            else:
                L.append("> O buffer trabalhou e drenou: houve queda de link "
                         "e os dados do período foram entregues com atraso, "
                         "não perdidos. É exatamente o comportamento que o "
                         "store-and-forward existe para produzir.")
        L.append("")

    if base:
        L.append("## Comparação com a linha de base LoRa")
        L.append("")
        L.append(f"Linha de base: `{base['nome']}`, gravada no caminho "
                 "anterior, com transmissão por LoRa.")
        L.append("")
        ganho = None
        if base.get("taxa_hz") and d.get("taxa_hz"):
            ganho = d["taxa_hz"] / base["taxa_hz"]
        L.append(tabela([
            ("Taxa LoRa", f(base.get("taxa_hz"), 2, " records/s")),
            ("Taxa ESP-NOW", f(d.get("taxa_hz"), 2, " records/s")),
            ("Ganho de taxa", f(ganho, 1, "×") if ganho else "—"),
            ("Intervalo mediano LoRa", f(base.get("iv_p50"), 1, " ms")),
            ("Intervalo mediano ESP-NOW", f(d.get("iv_p50"), 1, " ms")),
            ("Jitter LoRa", f(base.get("iv_sd"), 1, " ms")),
            ("Jitter ESP-NOW", f(d.get("iv_sd"), 1, " ms")),
            ("Records na base", f(base.get("n"), 0)),
            ("Duração da base", f(base.get("dur_s"), 1, " s")),
        ]))
        L.append("")
        if ganho and base.get("iv_p50") and d.get("iv_p50"):
            nyq_b = 1000.0 / (2 * base["iv_p50"])
            nyq_n = 1000.0 / (2 * d["iv_p50"])
            L.append(f"Em termos do que se consegue analisar: com intervalo "
                     f"mediano de {base['iv_p50']:.0f} ms, o caminho LoRa "
                     f"resolvia conteúdo até cerca de {nyq_b:.1f} Hz pelo "
                     f"critério de Nyquist. Com {d['iv_p50']:.0f} ms, o "
                     f"caminho ESP-NOW chega a cerca de {nyq_n:.1f} Hz — a "
                     f"diferença entre ver que a suspensão trabalhou e ver "
                     f"como ela trabalhou.")
            L.append("")

    L.append("---")
    L.append("")
    L.append(f"Relatório gerado em {datetime.now().isoformat(timespec='seconds')}.")
    return "\n".join(L)


# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Relatório de desempenho do link a partir de um log CSV.")
    ap.add_argument("csv", type=Path, nargs="+",
                    help="log(s) da sessão a analisar")
    ap.add_argument("--baseline", type=Path, default=None,
                    help="log do caminho LoRa, para comparação")
    ap.add_argument("-o", "--out", type=Path, default=None,
                    help="arquivo de saída (padrão: imprime na tela)")
    ap.add_argument("--alcance", action="store_true",
                    help="modo ensaio de alcance: segmenta o log por janela "
                         "de tempo e classifica paradas por estabilidade de RSSI")
    ap.add_argument("--janela", type=float, default=15.0,
                    help="duração da janela em segundos no modo alcance (padrão 15)")
    ap.add_argument("--tabela", action="store_true",
                    help="monta a tabela do ensaio de alcance a partir de vários "
                         "logs, usando o rótulo do nome de cada arquivo como marca")
    a = ap.parse_args()

    # ---- modo tabela: um log por marca ------------------------------------
    if a.tabela:
        itens: list[tuple[str, dict[str, Any]]] = []
        for c in a.csv:
            if not c.exists():
                print(f"aviso: {c} não existe, ignorando", file=sys.stderr)
                continue
            lin = ler(c)
            if not lin:
                print(f"aviso: {c} está vazio, ignorando", file=sys.stderr)
                continue
            itens.append((rotulo_do_nome(c), analisar(lin, c.name)))
        if not itens:
            print("erro: nenhum log válido", file=sys.stderr)
            return 1
        itens.sort(key=lambda it: ordem_rotulo(it[0]))
        texto = render_tabela(itens)
        if a.out:
            a.out.parent.mkdir(parents=True, exist_ok=True)
            a.out.write_text(texto + "\n", encoding="utf-8")
            print(f"tabela de {len(itens)} marcas escrita em {a.out}")
        else:
            print(texto)
        return 0

    alvo = a.csv[0]
    if not alvo.exists():
        print(f"erro: {alvo} não existe", file=sys.stderr)
        return 1

    linhas = ler(alvo)
    if not linhas:
        print(f"erro: {alvo} está vazio", file=sys.stderr)
        return 1
    d = analisar(linhas, alvo.name)

    base = None
    if a.baseline:
        if not a.baseline.exists():
            print(f"aviso: linha de base {a.baseline} não existe, ignorando",
                  file=sys.stderr)
        else:
            lb = ler(a.baseline)
            if lb:
                base = analisar(lb, a.baseline.name)

    if a.alcance:
        segs = segmentar(linhas, a.janela)
        if not segs:
            print("erro: log curto demais para segmentar", file=sys.stderr)
            return 1
        texto = render_alcance(d, segs, a.janela)
    else:
        texto = render(d, base)
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(texto + "\n", encoding="utf-8")
        print(f"relatório escrito em {a.out}")
    else:
        print(texto)
    return 0


if __name__ == "__main__":
    sys.exit(main())
