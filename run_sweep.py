#!/usr/bin/env python3
"""
Barrido esférico automatizado para el estudio de arrastre del dron en FluidX3D.
Es importante compilar con ./make.sh con la función main para el barrido antes de ejecutar este script
Genera una malla de orientaciones (elevación, azimut) sobre la esfera de
direcciones de flujo, corre el binario de FluidX3D una vez por orientación,
y organiza los CSV resultantes en carpetas etiquetadas.

Explota la simetría del cuerpo:
  - simetría especular izquierda-derecha -> azimut solo 0..180 (el resto es espejo)
  - asimetría Frente-Reverso                  -> elevación completa -90..90
  Esto va a variar según la geometría de tu dron

Uso:
    python3 run_sweep.py --step 15 --exe ./bin/FluidX3D --outdir Sphere
    python3 run_sweep.py --step 15 --exe ./bin/FluidX3D --outdir Sphere --angles-file bin/sweep_angles.txt --resume     # continúa, salta casos ya hechos
    
   
"""

import argparse
import os
import shutil
import subprocess
import sys
import time

import numpy as np


def build_orientations(step):
    """Lista de (elev, azim) en grados sobre el dominio independiente.

    Simetría explotada:
      - espejo izquierda-derecha -> azimut 0..180 (el resto es espejo)

    Elevación COMPLETA -90..90: la asimetría frente-reverso del cuerpo (~10%, medida
    en el barrido de yaw) vive en el signo de la elevación. +elev pone la "nariz"
    a favor del flujo, -elev en contra, y dan Cd*A distintos. Por eso NO se
    reduce a 0..90 — la simetría arriba-abajo del cuerpo no intercambia frente-reverso.

    (Verificar con (90,90) vs (-90,90): si difieren ~10%, este dominio es el
    correcto; si coinciden, puede reducirse a elev 0..90.)"""
    elevs = np.arange(-90, 90 + 1, step)
    azims = np.arange(0, 180 + 1, step)
    orientations = [(int(e), int(a)) for e in elevs for a in azims]
    return orientations


def case_tag(elev, azim):
    return f"e{elev:+03d}_a{azim:+03d}"


def csv_is_complete(path, t_final=1.5, tol=0.02):
    """True si el CSV existe y llegó al final de la simulación (~t_final s).
    Lee la última línea y comprueba la columna si_time. Evita tratar como
    'hecho' un caso interrumpido a media corrida."""
    if not os.path.isfile(path):
        return False
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            # leer el último ~2KB para encontrar la última línea no vacía
            f.seek(max(0, size - 2048))
            tail = f.read().decode("utf-8", errors="ignore").strip().splitlines()
        if len(tail) < 2:               # solo cabecera o vacío
            return False
        last = tail[-1].split(",")
        si_time = float(last[1])        # columna si_time
        return si_time >= t_final - tol
    except Exception:
        return False


def expected_csv(exe_dir, elev, azim):
    # debe coincidir con el nombre que escribe el setup.cpp parchado
    return os.path.join(exe_dir, "export", f"rb_state_{case_tag(elev, azim)}.csv")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=int, default=15, help="paso angular en grados")
    ap.add_argument("--exe", default="./bin/FluidX3D", help="ruta al binario")
    ap.add_argument("--outdir", default="Sphere", help="carpeta destino de los CSV")
    ap.add_argument("--resume", action="store_true",
                    help="saltar casos cuyo CSV destino ya existe")
    ap.add_argument("--dry-run", action="store_true",
                    help="solo listar los casos, no correr")
    ap.add_argument("--angles-file", default=None,
                    help="si tu setup lee de archivo en vez de argv, su ruta "
                         "(ej. bin/sweep_angles.txt)")
    args = ap.parse_args()

    exe = os.path.abspath(args.exe)
    exe_dir = os.path.dirname(exe)
    os.makedirs(args.outdir, exist_ok=True)

    orientations = build_orientations(args.step)
    print(f"Malla paso {args.step}deg: {len(orientations)} orientaciones")
    est_min = len(orientations) * 7  # asumir ~7 min/caso a 4000mb
    print(f"Tiempo estimado: ~{est_min/60:.1f} h a 4000mb (~7 min/caso)\n")

    if args.dry_run:
        for e, a in orientations:
            print(f"  {case_tag(e, a)}")
        return

    if not os.path.isfile(exe):
        sys.exit(f"ERROR: no se encontró el binario en {exe}")

    done, skipped, failed = 0, 0, []
    t_start = time.time()

    for i, (elev, azim) in enumerate(orientations, 1):
        tag = case_tag(elev, azim)
        dest = os.path.join(args.outdir, tag, "rb_state.csv")
        if args.resume and csv_is_complete(dest):
            skipped += 1
            continue
        # si existe un destino pero está incompleto (caso interrumpido), lo
        # borramos para rehacerlo limpio
        if os.path.isfile(dest) and not csv_is_complete(dest):
            print(f"   (caso previo incompleto en {tag}, rehaciendo)")
            os.remove(dest)
        # limpiar cualquier CSV truncado que haya quedado en export/ de una
        # corrida interrumpida, para que no contamine este caso
        stale = expected_csv(exe_dir, elev, azim)
        if os.path.isfile(stale):
            os.remove(stale)

        print(f"[{i}/{len(orientations)}] {tag}  "
              f"(elapsed {(time.time()-t_start)/60:.0f} min)", flush=True)

        # opción A: pasar ángulos por argv (setup parametrizado con main_arguments)
        cmd = [exe, str(elev), str(azim)]
        # opción B: si tu setup lee de archivo, escribirlo antes de lanzar
        if args.angles_file:
            with open(args.angles_file, "w") as f:
                f.write(f"{elev} {azim}\n")
            cmd = [exe]

        try:
            subprocess.run(cmd, cwd=exe_dir, check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            print(f"   FALLÓ (return {e.returncode})")
            failed.append(tag)
            continue

        # mover el CSV producido a la carpeta destino etiquetada
        src = expected_csv(exe_dir, elev, azim)
        if not os.path.isfile(src):
            # fallback: el setup podría seguir usando el nombre antiguo
            alt = os.path.join(exe_dir, "export", "rb_state.csv")
            src = alt if os.path.isfile(alt) else src
        if os.path.isfile(src):
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            shutil.move(src, dest)
            if csv_is_complete(dest):
                done += 1
            else:
                print(f"   AVISO: {tag} se movió pero el CSV parece incompleto "
                      f"(no llegó a 1.5 s)")
                failed.append(tag)
        else:
            print(f"   AVISO: no se encontró el CSV de salida ({src})")
            failed.append(tag)

    dt = (time.time() - t_start) / 60
    print(f"\nListo. corridos={done} saltados={skipped} fallidos={len(failed)} "
          f"en {dt:.0f} min")
    if failed:
        print("Fallidos:", ", ".join(failed))


if __name__ == "__main__":
    main()
