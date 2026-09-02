# Evaluación

Esta carpeta contiene las rutas, los resultados y los scripts de posprocesamiento empleados para evaluar el módulo de toma de decisiones desarrollado.

## Protocolo de evaluación

La evaluación sigue un protocolo reducido y adaptado basado en Bench2Drive. Se emplearon CARLA 0.9.15 y las definiciones de rutas de [Bench2Drive 0.0.3](https://github.com/Thinklab-SJTU/Bench2Drive/tree/2645714eb1f3a100217928dd113093cae0779f36).

El conjunto está formado por 44 rutas seleccionadas del banco original, identificadas internamente mediante valores comprendidos entre 0 y 43. Se realizó una ejecución por ruta y no se incluyeron escenarios de Town13.

Este procedimiento no reproduce el protocolo oficial completo de Bench2Drive. Por tanto, los resultados obtenidos caracterizan el comportamiento del módulo dentro del subconjunto evaluado y no son directamente comparables con los resultados de la clasificación oficial del banco de pruebas.

Los parámetros del módulo se ajustaron antes de seleccionar las 44 rutas definitivas. Para ello se utilizaron ensayos informales no incluidos en esta evaluación y realizados sobre rutas diferentes, evitando adaptar los parámetros específicamente a los escenarios evaluados.

## Estructura

```text
evaluation/
├── README.md
├── routes/
│   ├── bench2drive44_selected.xml
│   └── bench2drive44_mapping.csv
├── scripts/
│   ├── merge_jsons_44.py
│   └── b2d44_metrics_from_results_json.py
└── results/
    ├── individual/
    │   ├── results_00.json
    │   ├── ...
    │   └── results_43.json
    ├── merged_44_results.json
    ├── merge_report.csv
    ├── global_metrics_44.csv
    ├── ability_metrics_44.csv
    └── per_route_metrics_44.csv
```

### Rutas

* `bench2drive44_selected.xml`: contiene las 44 rutas ejecutables, renumeradas del 0 al 43.
* `bench2drive44_mapping.csv`: relaciona cada identificador interno con la ruta original de Bench2Drive, el mapa, el tipo de escenario y las capacidades evaluadas.

La selección definitiva incluye las siguientes sustituciones respecto a una versión preliminar:

* ID 22: `bench2drive_37`.
* ID 23: `bench2drive_185`.
* ID 36: `bench2drive_122`.
* ID 39: `bench2drive_135`.

### Resultados

* `individual/`: contiene la salida original de cada una de las 44 ejecuciones.
* `merged_44_results.json`: reúne los registros individuales en un único archivo.
* `merge_report.csv`: resume la integración de los archivos y permite comprobar que están presentes todos los identificadores.
* `global_metrics_44.csv`: contiene las métricas globales.
* `ability_metrics_44.csv`: presenta los resultados agrupados por capacidad.
* `per_route_metrics_44.csv`: recoge la trazabilidad y las métricas de cada ruta.

Una ruta puede estar asociada a varias capacidades. Por ello, los subconjuntos utilizados para calcular las métricas por capacidad no son mutuamente excluyentes.

## Resultados globales

| Métrica                            |   Valor |
| ---------------------------------- | ------: |
| Rutas evaluadas                    |      44 |
| Driving Score (DS)                 |   71,92 |
| Route Completion (RC)              | 89,89 % |
| Infraction Penalty (IP)            |  0,7984 |
| Success Rate (SR)                  | 43,18 % |
| Rutas perfectas                    |      19 |
| Rutas completadas con penalización |      18 |
| Rutas fallidas                     |       7 |

Para calcular la tasa de éxito, una ruta debe finalizar y no registrar infracciones relevantes. Las infracciones de velocidad mínima no se consideran bloqueantes para este indicador.

El campo `success_strict` de `merge_report.csv` emplea un criterio de comprobación más restrictivo: estado `Perfect`, finalización del 100 %, penalización igual a 1 y ausencia total de infracciones. En este conjunto, ambos criterios proporcionan 19 rutas exitosas.

## Reproducción del posprocesamiento

Los scripts utilizan únicamente la biblioteca estándar de Python. El archivo `merge_jsons_44.py` requiere Python 3.10 o posterior. Este posprocesamiento puede ejecutarse en un entorno independiente del utilizado para CARLA.

Desde la raíz del repositorio, los resultados individuales pueden combinarse mediante:

```bash
python evaluation/scripts/merge_jsons_44.py \
    --input evaluation/results/individual \
    --output evaluation/results/merged_44_results.json \
    --report evaluation/results/merge_report.csv \
    --expected 44
```

A continuación, las métricas se calculan mediante:

```bash
python evaluation/scripts/b2d44_metrics_from_results_json.py \
    --json evaluation/results/merged_44_results.json \
    --mapping evaluation/routes/bench2drive44_mapping.csv \
    --out evaluation/results
```

El segundo script genera las métricas globales, las métricas por capacidad y los resultados desagregados por ruta. También crea `normalized_results.json` como copia normalizada intermedia del archivo combinado.
