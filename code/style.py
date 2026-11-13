from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FIG_DIR = Path(__file__).resolve().parent.parent / "report" / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)

HALF = (3.6, 2.6)   # two figures side by side
FULL = (7.2, 3.0)

plt.rcParams.update({
    "font.size": 9,
    "axes.titlesize": 9.5,
    "axes.labelsize": 9,
    "legend.fontsize": 7.5,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "lines.linewidth": 1.5,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
})


def save(fig, name):
    path = FIG_DIR / f"{name}.png"
    fig.savefig(path)
    plt.close(fig)
    print(f"saved {path.relative_to(FIG_DIR.parent.parent)}")
