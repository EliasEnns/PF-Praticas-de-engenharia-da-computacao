#!/usr/bin/env python3
"""Carimba o número de página no canto superior direito, padrão ABNT.

A build do wkhtmltopdf disponível não renderiza cabeçalho, então a numeração
entra depois, por sobreposição. Posição: 2 cm da borda superior e 2 cm da
direita, alinhado à direita, Helvetica 10 pt.
"""
import io, sys
from pypdf import PdfReader, PdfWriter
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4

CM = 28.3465

def numerar(entrada, saida, pular_primeira=False):
    base = PdfReader(entrada)
    out = PdfWriter()
    w, h = A4
    for i, pag in enumerate(base.pages, start=1):
        if not (pular_primeira and i == 1):
            buf = io.BytesIO()
            c = canvas.Canvas(buf, pagesize=A4)
            c.setFont("Helvetica", 10)
            c.drawRightString(w - 2*CM, h - 2*CM, str(i))
            c.save()
            buf.seek(0)
            pag.merge_page(PdfReader(buf).pages[0])
        out.add_page(pag)
    with open(saida, "wb") as f:
        out.write(f)
    return len(base.pages)

if __name__ == "__main__":
    n = numerar(sys.argv[1], sys.argv[2])
    print(f"{n} páginas numeradas em {sys.argv[2]}")
