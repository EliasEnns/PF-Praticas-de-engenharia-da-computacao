# Como regerar o relatório em PDF

O PDF é gerado a partir de `relatorio.html`, que é a fonte editável. As figuras
estão em `fig/` como PNG a 2× e os originais vetoriais em `../figuras/` como SVG.

```
wkhtmltopdf --enable-local-file-access --page-size A4 \
  --margin-top 18mm --margin-bottom 16mm --margin-left 17mm --margin-right 17mm \
  --footer-font-size 8 --footer-right "[page]/[topage]" --footer-spacing 6 \
  relatorio.html ../Relatorio_N1C_Telemetria_ESP32_ESPNOW.pdf
```

## Quando as distâncias forem corrigidas pelas coordenadas

Os rótulos de distância aparecem em três lugares e precisam ser trocados nos três:

1. a tabela da seção 6.3 em `relatorio.html`;
2. o texto da seção 6.3 e da 6.4, onde "80 m" nomeia o episódio do buffer;
3. a figura 2, que é gerada por script — os rótulos estão na lista `MARCAS`.

Se a ordem relativa das marcas mudar, vale rodar de novo:

```
python ../../tools/link_report.py ../../Dashboard/logs/telemetria_*m.csv --tabela
```

e conferir a tabela contra a do relatório antes de publicar.
