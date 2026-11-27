#!/bin/sh
# Пересобрать плату-носитель: генерация (KiCad 6, для импорта в EasyEDA),
# DRC в KiCad и картинки для лазера. Нужен KiCad 8+ (kicad-cli) и Python 3,
# для PNG — PyMuPDF (pip install pymupdf).
#   sh hardware/fc-carrier-h743/build.sh
set -e
cd "$(dirname "$0")"
X="${KICAD_CLI:-$LOCALAPPDATA/Programs/KiCad/10.0/bin/kicad-cli.exe}"
B=openplane_carrier
python gen_board.py $B
"$X" pcb drc --refill-zones --severity-error --severity-warning --exit-code-violations -o drc.rpt $B.kicad_pcb
rm -f drc.rpt $B.kicad_prl
SVG="pcb export svg --mode-single --page-size-mode 2 --exclude-drawing-sheet"
"$X" $SVG --check-zones --black-and-white --negative --mirror --drill-shape-opt 1 -l B.Cu,Edge.Cuts \
    -o laser_bottom_copper_MIRRORED_NEGATIVE.svg $B.kicad_pcb
"$X" $SVG --black-and-white -l F.SilkS,Edge.Cuts -o laser_top_marking.svg $B.kicad_pcb
"$X" $SVG --check-zones --mirror -l B.Cu,B.Fab,Edge.Cuts -o assembly_bottom.svg $B.kicad_pcb
"$X" $SVG --check-zones -l B.Cu,F.SilkS,F.Fab,Edge.Cuts -o overview.svg $B.kicad_pcb
python - <<'EOF'
import pymupdf
for n, dpi in [("laser_bottom_copper_MIRRORED_NEGATIVE", 1200), ("laser_top_marking", 1200),
               ("assembly_bottom", 400), ("overview", 230)]:
    pymupdf.open(n + ".svg")[0].get_pixmap(dpi=dpi, alpha=False).save(n + ".png")
EOF
rm -f assembly_bottom.svg overview.svg
echo "готово: DRC чистый, картинки обновлены"
