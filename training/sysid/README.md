# Fase 0 — Calibración (system-ID) de FIRASim

Mide la dinámica **real** de FIRASim para calibrar el simulador de entrenamiento del
Intento 3 (control `(v, ω)`). Ver el plan completo en
[`../PLAN_INTENTO3.md`](../PLAN_INTENTO3.md).

## Por qué
Con control `(v, ω)` el gap sim↔FIRASim deja de ser de "ejecutor" (path-planning UVF) y
pasa a ser **solo de física**. Esta carpeta mide esa física (vel. máx, aceleración,
fricción, pelota) para que el sim de entrenamiento la replique y el domain randomization
se centre en valores **medidos**, no adivinados.

## Requisitos
- FIRASim corriendo (defaults del engine: comandos UDP `127.0.0.1:20011`, visión
  multicast `224.0.0.1:10002`).
- `pip install grpcio-tools` (una vez, para generar los bindings protobuf).
- `numpy` (ya está en `requirements.txt`).

## Pasos
```bash
# 1) Generar los bindings protobuf FIRA (una vez)
python gen_protos.py

# 2) Recolectar datos contra FIRASim (corré FIRASim primero)
python firasim_sysid.py
#    Si tu build reporta en milímetros:  python firasim_sysid.py --field-units mm

# 3) Ajustar y producir sim_calibration.json
python fit_sysid.py
```

El paso 2 hace un **preflight**: teleporta el robot a (-0.5, 0) y te dice qué posición
reporta FIRASim. Si ves un número grande (~ -500), tu FIRASim está en **mm** → reintentá
con `--field-units mm`.

## Qué produce
- `data/lin_step_v*.csv`, `data/ang_step_w*.csv`, `data/ball_friction.csv` — respuestas crudas.
- `sim_calibration.json` — parámetros medidos:
  `v_max`, `omega_max`, `lin_accel`, `ang_accel`, `lin_decel`, `ang_decel`, `ball_decel`.

## Cómo se usa después (Fase 1/3)
En `vsss_rl/soccer_env.py`, el integrador de `(v, ω)` usa estos valores como topes y
rampas de aceleración, y el DR los multiplica por `U(0.8, 1.2)` por episodio.

## Notas / posibles ajustes (no testeado contra un FIRASim vivo)
- Si FIRASim no acepta `Replacement` por el puerto de comandos, el teleport no posiciona;
  verificá en la GUI de FIRASim. (El formato es el `Packet{replace}` estándar de FIRA.)
- Los `v*.csv` arrancan al robot en `x=-0.55` mirando `+x`; si tu campo es más chico,
  bajá las duraciones de los segmentos en `firasim_sysid.py`.
- Las velocidades se leen del campo `vx/vy/vorientation` que FIRASim publica; si tu build
  no las publica, el fitter las puede derivar de la posición (ajuste menor en `fit_sysid.py`).
