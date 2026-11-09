#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teste de contrato entre o firmware do gateway e o dashboard.
============================================================

Existe exatamente um ponto em que o firmware em C e o dashboard em Python
precisam concordar byte a byte: a linha que o gateway escreve na porta COM.
Se as duas pontas divergirem, o sintoma no laboratório é ruim de diagnosticar
-- o dashboard simplesmente descarta a linha em silêncio, porque foi feito
assim de propósito para tolerar quadro truncado durante o boot do MCU, e a
tela fica parada sem nenhuma mensagem de erro.

Este teste fecha esse risco sem precisar de hardware:

  1. tools/test_bajalink.cpp compila a função REAL do firmware
     (bl_json_decorate, em baja_espnow_rx/bl_json.h) e gera 600 linhas de
     saída com o emulador de telemetria, variando os campos do link para
     cobrir negativos, zeros e valores grandes.
  2. Este script passa essas linhas pelo parser REAL do dashboard
     (SerialManager._process_line) e verifica que cada campo chegou do outro
     lado com o valor certo, que o CSV ganhou as colunas do link, e que os
     construtores de figura e de cartões aceitam o resultado.

Também verifica a compatibilidade retroativa: uma linha do caminho LoRa
antigo, sem o objeto "link", tem de continuar sendo processada normalmente.

Uso:
    g++ -std=c++17 -O1 -I tools/stub -I baja_espnow_tx -I baja_espnow_rx \\
        tools/test_bajalink.cpp -o /tmp/test_bajalink && /tmp/test_bajalink
    python tools/test_formato.py
"""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import tempfile
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
DASH = RAIZ / "Dash teste Claude" / "telemetria_baja.py"

# Arquivo gerado pelo test_bajalink. Se ele nao tiver sido rodado (ou nao
# houver compilador C++ na maquina), cai na amostra versionada, que foi gerada
# pela mesma funcao do firmware. O teste continua valendo; so nao reflete
# alteracoes recentes no bl_json.h.
LINHAS_GERADAS = Path("/tmp/bajalink_linhas.jsonl")
LINHAS_AMOSTRA = RAIZ / "tools" / "amostra_linhas.jsonl"

falhas = 0


def check(ok: bool, what: str) -> None:
    global falhas
    print(f"  [{'ok' if ok else 'FALHOU'}] {what}")
    if not ok:
        falhas += 1


def carregar_dashboard():
    """Importa o dashboard sem subir o servidor HTTP."""
    spec = importlib.util.spec_from_file_location("telemetria_baja", DASH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"nao consegui carregar {DASH}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["telemetria_baja"] = mod
    spec.loader.exec_module(mod)
    return mod


def main() -> int:
    print("=== contrato firmware <-> dashboard ===")

    if not DASH.exists():
        print(f"  [FALHOU] dashboard nao encontrado em {DASH}")
        return 1
    if LINHAS_GERADAS.exists():
        fonte = LINHAS_GERADAS
        origem = "geradas agora pelo test_bajalink"
    elif LINHAS_AMOSTRA.exists():
        fonte = LINHAS_AMOSTRA
        origem = "amostra versionada (test_bajalink nao foi rodado)"
    else:
        print("  [FALHOU] nenhum arquivo de linhas encontrado")
        return 1

    print("\ncarregando o dashboard")
    mod = carregar_dashboard()
    check(True, "modulo importado sem subir servidor")
    check(len(mod.LINK_KEYS) > 0, "LINK_FIELDS declarado")

    linhas = [l for l in fonte.read_text(encoding="utf-8").splitlines() if l.strip()]
    print(f"\n{len(linhas)} linhas do firmware ({origem})")
    check(len(linhas) > 100, "arquivo de linhas tem conteudo")

    # Toda linha tem de ser JSON valido com os dois objetos anexados.
    objs = []
    ok_json = True
    for l in linhas:
        try:
            o = json.loads(l)
            objs.append(o)
        except json.JSONDecodeError:
            ok_json = False
            break
    check(ok_json, "toda linha e JSON valido")
    check(all("link" in o for o in objs), "toda linha traz o objeto link")
    check(all("tele" in o for o in objs), "toda linha traz o objeto tele")

    # Todas as chaves que o dashboard conhece existem no fio, e vice-versa:
    # uma chave a mais no firmware seria dado jogado fora; uma a menos, campo
    # vazio no dashboard. As duas direcoes importam.
    no_fio = set(objs[0]["link"].keys())
    no_dash = set(mod.LINK_KEYS)
    check(no_dash - no_fio == set(),
          f"dashboard nao espera campo inexistente (faltando: {sorted(no_dash - no_fio)})")
    check(no_fio - no_dash == set(),
          f"firmware nao manda campo ignorado (sobrando: {sorted(no_fio - no_dash)})")

    # Esquema de telemetria: as variaveis do catalogo tem de estar no record.
    faltando = [k for k in mod.VAR_KEYS if k not in objs[0]]
    check(not faltando, f"record traz as variaveis do catalogo (faltando: {faltando})")

    print("\npassando pelo parser do dashboard")
    with tempfile.TemporaryDirectory() as tmp:
        mod.CSV_DIR = Path(tmp)
        sm = mod.serial_mgr
        mod.data_store.reset_session()
        sm._logging_enabled = True
        sm._logging_paused = False
        sm._open_csv()
        for l in linhas:
            sm._process_line(l)
        csv_path = sm._csv_path
        sm._close_csv()

        stats = mod.data_store.snapshot_stats()
        check(stats["total_rx"] == len(linhas),
              f"todos os {len(linhas)} records entraram no buffer")

        ultimo = stats["last_packet"] or {}
        check(isinstance(ultimo.get("link"), dict), "objeto link preservado no pacote")

        # Achatamento: cada campo do link virou link_<chave> numerico, com o
        # mesmo valor do fio.
        fio = objs[-1]["link"]
        erradas = []
        for k in mod.LINK_KEYS:
            v = ultimo.get(f"link_{k}")
            if v is None or not math.isclose(float(v), float(fio[k]), rel_tol=1e-6):
                erradas.append(k)
        check(not erradas, f"achatamento preserva os valores (erradas: {erradas})")

        # Series: o grafico do painel le daqui.
        rssi = mod.data_store.get_recent_samples("link_rssi", 200, use_filtered=False)
        pdr = mod.data_store.get_recent_samples("link_pdr", 200, use_filtered=False)
        check(len(rssi) > 0 and len(pdr) > 0, "series link_rssi e link_pdr disponiveis")
        check(all(-120.0 <= v <= 0.0 for _, v in rssi), "RSSI em faixa plausivel")
        check(all(0.0 <= v <= 100.0 for _, v in pdr), "PDR entre 0 e 100")

        # CSV
        check(csv_path is not None and csv_path.exists(), "CSV gravado")
        if csv_path and csv_path.exists():
            texto = csv_path.read_text(encoding="utf-8").splitlines()
            cab = texto[0].split(",")
            check(all(c in cab for c in mod.LINK_CSV_COLS),
                  "cabecalho do CSV tem as colunas do link")
            check(len(texto) == len(linhas) + 1,
                  f"CSV tem {len(linhas)} linhas de dados")
            # Nenhuma coluna de link vazia num log do caminho ESP-NOW.
            idx = {c: i for i, c in enumerate(cab)}
            amostra = texto[-1].split(",")
            vazias = [c for c in mod.LINK_CSV_COLS if not amostra[idx[c]].strip()]
            check(not vazias, f"colunas do link preenchidas (vazias: {vazias})")

    print("\nconstrutores de UI")
    fig = mod.build_link_figure(rssi, pdr, [(1.0, 10.0), (2.0, 12.0)])
    check(len(fig.data) == 3, "figura do link tem as tres series")
    tiles = mod.build_link_tiles(ultimo.get("link") or {})
    check(len(tiles) > 8, "cartoes do link construidos")
    check(len(mod.build_link_tiles({})) == 1,
          "sem objeto link, mostra aviso em vez de quebrar")

    print("\ncompatibilidade com o caminho LoRa antigo")
    mod.data_store.reset_session()
    antiga = json.dumps({
        "ts": 2002849, "seq": 1670, "roda_fr": 0, "roda_rr": 0,
        "ax": -1, "ay": -2, "az": 14, "gx": 0, "gy": -1, "gz": -1,
        "bateria_mv": 12752, "motor_temp": 27, "cvt_temp": 22,
        "combustivel": 3313, "estop": 0,
        "tele": {"rx": 1670, "lost": 0, "loss_pct": 0.0, "rst": 1},
    })
    sm._logging_enabled = False
    sm._process_line(antiga)
    st = mod.data_store.snapshot_stats()
    p = st["last_packet"] or {}
    check(st["total_rx"] == 1, "record sem objeto link e processado")
    check("link" not in p, "nenhum campo de link inventado")
    check(p.get("tele", {}).get("rx") == 1670, "tele do receptor LoRa preservado")

    # Robustez: o parser nao pode cair com lixo na porta.
    for ruim in ('', '   ', '{"ts":1,', 'nao e json', '[]', 'null',
                 '{"ts":1,"link":"isto nao e objeto"}',
                 '{"ts":1,"link":{"rssi":"abc"}}'):
        sm._process_line(ruim)
    check(True, "lixo na porta nao derruba o parser")

    print(f"\n=== {'todos os testes passaram' if falhas == 0 else 'HOUVE FALHAS'} ===")
    return 1 if falhas else 0


if __name__ == "__main__":
    sys.exit(main())
