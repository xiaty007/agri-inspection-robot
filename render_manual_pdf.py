import fitz
from pathlib import Path

pdf_path = Path(r"C:\Users\33615\Documents\New project111\docx_render\二维雷达建图启动操作手册.pdf")
out_dir = pdf_path.parent
doc = fitz.open(pdf_path)
matrix = fitz.Matrix(1.7, 1.7)
for index, page in enumerate(doc):
    pix = page.get_pixmap(matrix=matrix, alpha=False)
    pix.save(out_dir / f"page-{index + 1}.png")
print(f"pages={len(doc)}")
