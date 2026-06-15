# Modificación de `setup.cpp` para barrido esférico automatizado

## Objetivo
Eliminar las recompilaciones: en vez de editar a mano la línea de rotación y
recompilar para cada caso, el ejecutable lee dos ángulos (elevación, azimut) de
la línea de comandos. Compilas **una sola vez** y el script de barrido corre el
mismo binario con distintos ángulos.

## Cambio 1 — leer argumentos al inicio de `main_setup()`

FluidX3D usa la variable global `main_arguments` (un `vector<string>`) que captura
los argumentos de la línea de comandos. Añade esto justo al inicio del caso del
dron (después de la línea 648, antes de `const uint3 lbm_N = ...`):

```cpp
void main_setup() { // Drone drag force study; ...
    // ---- barrido esférico: leer elevación y azimut de la línea de comandos ----
    // uso:  ./FluidX3D <elev_deg> <azim_deg>
    // si no se pasan argumentos, usa 0,0 (cuerpo en orientación de referencia)
    float elev_deg = 0.0f, azim_deg = 0.0f;
    if(main_arguments.size() >= 1u) elev_deg = to_float(main_arguments[0]);
    if(main_arguments.size() >= 2u) azim_deg = to_float(main_arguments[1]);
    // ---------------------------------------------------------------------------
```

(Si `main_arguments` no está disponible en tu versión, ver la nota al final.)

## Cambio 2 — reemplazar la línea de rotación (línea 673)

Quita:
```cpp
const float3x3 rotation = float3x3(float3(1, 0, 0), radians(0.0f))*float3x3(float3(0, 0, 1), radians(60.0f));
```

Pon:
```cpp
// Rotación compuesta que apunta el cuerpo a la dirección (elev, azim) sobre la
// esfera, relativa al flujo fijo en +Y.
//   azimut: giro alrededor de Y (eje del flujo)  -> barre izquierda/derecha
//   elevación: giro alrededor de X               -> barre arriba/abajo (proa/popa)
// El orden importa: primero elevación en el marco del cuerpo, luego azimut.
const float3x3 rotation =
      float3x3(float3(0, 1, 0), radians(azim_deg))   // azimut alrededor de Y (flujo)
    * float3x3(float3(1, 0, 0), radians(elev_deg));  // elevación alrededor de X
```

## Cambio 3 — etiquetar el CSV de salida por orientación

Quita (líneas 683-684):
```cpp
char csv_speed_buf[32]; sprintf(csv_speed_buf, "%g", si_u);
const string csv = get_exe_path()+"export/rb_state_"+replace(string(csv_speed_buf), ".", "_")+"m_s.csv";
```

Pon:
```cpp
// nombre del CSV incluye la orientación, para no sobrescribir entre casos
char tag[64];
sprintf(tag, "e%+03d_a%+03d", (int)lround(elev_deg), (int)lround(azim_deg));
const string csv = get_exe_path()+"export/rb_state_"+string(tag)+".csv";
```

Esto produce archivos como `rb_state_e+00_a+60.csv`, `rb_state_e-15_a+30.csv`,
etc., uno por orientación, todos en `export/`.

## Nota: si `main_arguments` no existe en tu versión

Algunas versiones de FluidX3D no exponen `main_arguments`. Alternativa robusta:
leer los ángulos de un archivo de texto que el script reescribe antes de cada
corrida. Reemplaza el Cambio 1 por:

```cpp
float elev_deg = 0.0f, azim_deg = 0.0f;
{
    std::ifstream fin(get_exe_path()+"sweep_angles.txt");
    if(fin.good()) fin >> elev_deg >> azim_deg;
}
```

y el script escribe `bin/sweep_angles.txt` con dos números antes de lanzar el
binario. (Requiere `#include <fstream>` arriba, que ya suele estar.)

## Verificación rápida tras compilar

Corre un caso conocido y compáralo con tu barrido anterior:
```
./FluidX3D 0 60      # debe reproducir tu caso roll=60 / la línea original
```
El `Cd*A` resultante debe coincidir (dentro del ruido) con el valor que ya tenías
para esa orientación, confirmando que la rotación parametrizada equivale a la
hardcodeada.
