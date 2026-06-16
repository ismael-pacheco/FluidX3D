"""
Análisis del barrido esférico de arrastre del dron.

Lee los CSV de Sphere/e±XX_a±YY/rb_state.csv, calcula Cd*A por orientación
(fuerza media de estado estacionario, sin depender de un área de referencia),
y produce:
  1. Mapa de calor 2D  Cd*A(elev, azim)  — fácil de leer
  2. Proyección sobre la media esfera real (cobertura geométrica)
  3. Tabla resumen: min/max, ubicación, y chequeos de consistencia

Dominio muestreado: elev -90..90, azim 0..180 (media esfera; la otra media se
reconstruye por simetría especular izquierda-derecha azim -> -azim).
"""

import glob
import os
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm

# -----------------------------
# Config
# -----------------------------
ROOT = "Sphere"
RHO = 1000.0
VELOCITY = 5.0
TRANSIENT_FRAC = 0.5
DRAG_COL = "force_y"


# -----------------------------
# Carga
# -----------------------------
def parse_tag(dirname):
    m = re.search(r"e([+-]?\d+)_a([+-]?\d+)", dirname)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2))


def cda_from_csv(path):
    Fy = pd.read_csv(path)[DRAG_COL].to_numpy()
    steady = Fy[int(len(Fy) * TRANSIENT_FRAC):].mean()
    return 2.0 * steady / (RHO * VELOCITY * VELOCITY)


def load_sphere(root):
    data = {}  # (elev, azim) -> Cd*A
    for d in glob.glob(os.path.join(root, "e*_a*")):
        tag = parse_tag(os.path.basename(d))
        if tag is None:
            continue
        csv = os.path.join(d, "rb_state.csv")
        if os.path.isfile(csv):
            try:
                data[tag] = cda_from_csv(csv)
            except Exception as e:
                print(f"[warn] {d}: {e}")
    return data


# -----------------------------
# Main
# -----------------------------
data = load_sphere(ROOT)
if not data:
    raise SystemExit(f"No se encontraron CSV en {ROOT}/e*_a*/rb_state.csv")

elevs = sorted(set(e for e, a in data))
azims = sorted(set(a for e, a in data))
print(f"Cargadas {len(data)} orientaciones: "
      f"elev {min(elevs)}..{max(elevs)}, azim {min(azims)}..{max(azims)}")

# matriz para el heatmap (filas=elev, columnas=azim)
Z = np.full((len(elevs), len(azims)), np.nan)
for (e, a), c in data.items():
    Z[elevs.index(e), azims.index(a)] = c

# resumen
vals = np.array(list(data.values()))
imin = min(data, key=data.get)
imax = max(data, key=data.get)
print(f"\nCd*A min = {data[imin]:.5f} en elev={imin[0]} azim={imin[1]}")
print(f"Cd*A max = {data[imax]:.5f} en elev={imax[0]} azim={imax[1]}")
print(f"rango: {data[imax]/data[imin]:.2f}x")

# chequeo proa-popa: comparar +elev vs -elev al mismo azim
print("\nAsimetría proa-popa (Cd*A[+e] vs Cd*A[-e], promedio sobre azim):")
for e in sorted(set(abs(x) for x in elevs)):
    if e == 0:
        continue
    pos = [data[(e, a)] for a in azims if (e, a) in data]
    neg = [data[(-e, a)] for a in azims if (-e, a) in data]
    if pos and neg:
        pm, nm = np.mean(pos), np.mean(neg)
        print(f"  |elev|={e:3d}: +{pm:.5f} vs -{nm:.5f}  "
              f"({100*(pm-nm)/(0.5*(pm+nm)):+.1f}%)")

# huecos (casos faltantes en la malla)
missing = [(e, a) for e in elevs for a in azims if (e, a) not in data]
if missing:
    print(f"\n{len(missing)} casos faltantes en la malla:")
    print("  " + ", ".join(f"e{e}_a{a}" for e, a in missing[:15])
          + (" ..." if len(missing) > 15 else ""))

# -----------------------------
# Figura 1: mapa de calor 2D
# -----------------------------
fig, ax = plt.subplots(figsize=(9, 6))
im = ax.imshow(Z, origin="lower", aspect="auto", cmap="viridis",
               extent=[min(azims) - 0.5, max(azims) + 0.5,
                       min(elevs) - 0.5, max(elevs) + 0.5],
               interpolation="nearest")
cb = fig.colorbar(im, ax=ax, label="$C_d \\cdot A$  (m$^2$)")
ax.set_xlabel("Azimut (deg)  —  giro alrededor del eje de flujo")
ax.set_ylabel("Elevación (deg)  —  0=canto, ±90=cara (proa/popa)")
ax.set_title("Arrastre sobre la media esfera de direcciones de flujo")
# marcar min y max
ax.scatter([imin[1]], [imin[0]], marker="v", color="cyan", s=80,
           edgecolors="k", label=f"min {data[imin]:.4f}", zorder=5)
ax.scatter([imax[1]], [imax[0]], marker="^", color="red", s=80,
           edgecolors="k", label=f"max {data[imax]:.4f}", zorder=5)
ax.legend(loc="upper right", fontsize=8)
plt.tight_layout()
plt.savefig("sphere_heatmap.png", dpi=150)
print("\nguardado sphere_heatmap.png")

# -----------------------------
# Figura 2: proyección sobre la media esfera 3D
# -----------------------------
# vector de dirección de flujo en ejes del cuerpo a partir de (elev, azim).
# Convención: elev rota sobre X, azim rota sobre Y (eje de flujo).
EE, AA = np.meshgrid(np.radians(elevs), np.radians(azims), indexing="ij")
# dirección de flujo unitaria parametrizada
xs = np.cos(EE) * np.sin(AA)
ys = np.sin(EE)
zs = np.cos(EE) * np.cos(AA)

fig2 = plt.figure(figsize=(9, 7))
ax2 = fig2.add_subplot(111, projection="3d")
# normalizar color
norm = (Z - np.nanmin(Z)) / (np.nanmax(Z) - np.nanmin(Z))
colors = cm.viridis(np.nan_to_num(norm))
surf = ax2.plot_surface(xs, ys, zs, facecolors=colors, rstride=1, cstride=1,
                        linewidth=0, antialiased=False, shade=False)
mappable = cm.ScalarMappable(cmap="viridis")
mappable.set_array(vals)
fig2.colorbar(mappable, ax=ax2, shrink=0.6, label="$C_d \\cdot A$ (m$^2$)")
ax2.set_xlabel("x"); ax2.set_ylabel("y (flujo)"); ax2.set_zlabel("z")
ax2.set_title("Cobertura: media esfera muestreada\n(la otra media es espejo izq-der)")
ax2.set_box_aspect((1, 1, 1))
plt.tight_layout()
plt.savefig("sphere_3d.png", dpi=150)
print("guardado sphere_3d.png")