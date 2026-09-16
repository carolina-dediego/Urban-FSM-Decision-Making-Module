# Módulo de toma de decisiones para navegación autónoma en entornos urbanos 

Módulo de toma de decisiones basado en máquinas de estados finitos para navegación autónoma urbana en el simulador CARLA.

Este repositorio contiene el código desarrollado para integrar una estrategia de decisión longitudinal y lateral en la arquitectura modular de navegación autónoma del grupo de investigación RobeSafe.

## Descripción

El módulo actúa como una capa intermedia de razonamiento entre la representación estructurada del entorno y los componentes encargados de controlar el movimiento del vehículo.

Sus funciones principales son:

- Interpretar la información proporcionada por la percepción y la infraestructura.
- Obtener distancias métricas e indicadores de circulación.
- Seleccionar la restricción que condiciona el comportamiento del vehículo.
- Determinar la velocidad de referencia.
- Evaluar el inicio de maniobras de adelantamiento.
- Generar las rutas utilizadas durante el adelantamiento y el retorno al carril original.

El módulo no constituye por sí solo un agente autónomo ejecutable de forma independiente. Su funcionamiento en lazo cerrado depende de las entradas y de las interfaces proporcionadas por la arquitectura de RobeSafe. No obstante, puede adaptarse a otras arquitecturas que proporcionen información equivalente.

## Organización funcional

El módulo se divide en tres componentes principales.

### Observador del entorno

El observador transforma las representaciones en perspectiva cenital y el resto de la información recibida en variables directamente utilizables por la lógica de decisión.

Entre sus resultados se encuentran:

- Distancia al vehículo precedente.
- Distancia a peatones o bicicletas situados en el carril.
- Distancias frontal y posterior en los carriles adyacentes.
- Presencia de carriles adyacentes y de sentido contrario.
- Distancia a semáforos y señales de STOP.
- Distancia a vehículos que atraviesan una intersección.
- Detección y distancia de obstáculos frontales.

### Planificador principal

El planificador principal selecciona la restricción activa y actualiza la máquina de estados longitudinal.

A partir de la situación observada, determina:

- El límite de velocidad asociado a la curvatura de la ruta.
- El estado longitudinal del vehículo.
- La velocidad objetivo.
- La velocidad de referencia filtrada.
- La solicitud de maniobra lateral.
- El indicador de urgencia asociado a la solicitud.

### Gestor de maniobras laterales

El gestor lateral recibe la solicitud generada por el planificador principal y controla el desarrollo de la maniobra mediante una segunda máquina de estados.

Este componente genera una copia modificada de la ruta de referencia para:

- Desplazar progresivamente el vehículo hacia el carril de adelantamiento.
- Mantener la trayectoria lateral durante la maniobra.
- Determinar cuándo se ha superado el obstáculo.
- Generar la transición de retorno al carril original.

## Máquinas de estados

El sistema utiliza dos máquinas de estados finitos coordinadas: una longitudinal y otra lateral.

### Máquina de estados longitudinal

La lógica longitudinal utiliza los siguientes estados:

| Estado | Función |
|---|---|
| `CRUISE` | Circulación sin restricciones próximas. |
| `COASTING` | Reducción anticipada de la velocidad ante una restricción lejana. |
| `FOLLOW` | Seguimiento de un actor precedente manteniendo una distancia de seguridad. |
| `BRAKING` | Frenado ante una restricción próxima. |
| `STOPPED` | Detención del vehículo. |

El estado longitudinal se selecciona en cada ciclo a partir de las condiciones observadas. La máquina puede pasar directamente al estado requerido sin recorrer obligatoriamente una secuencia fija de estados intermedios.

### Máquina de estados lateral

La lógica lateral utiliza los siguientes estados:

| Estado | Función |
|---|---|
| `GLOBAL` | Seguimiento de la ruta de referencia. |
| `OVERTAKE` | Ejecución y supervisión del adelantamiento. |
| `RETURN` | Retorno progresivo a la ruta original. |

La máquina lateral conserva el estado de la maniobra entre ciclos de ejecución y determina cuándo debe generarse una nueva ruta.

## Entradas principales

El módulo utiliza información procedente de los componentes de percepción, localización y planificación:

- Segmentación semántica en perspectiva cenital.
- Categorización de los carriles en la representación BEV.
- Estado y distancia de los semáforos.
- Información de señales de STOP.
- Distancia a la intersección.
- Detección de obstáculos frontales.
- Velocidad actual del vehículo.
- Posición del vehículo ego.
- Ruta local utilizada para analizar la geometría.
- Ruta de referencia empleada para generar las maniobras laterales.
- Límite de velocidad aplicable.

La lógica está diseñada para ejecutarse con un periodo nominal de `0.05 s`, equivalente a una frecuencia de `20 Hz`.

## Salidas

### Salidas del planificador principal

El planificador principal devuelve:

1. La velocidad de referencia, expresada en metros por segundo.
2. La orden lateral, codificada mediante `keep`, `left` o `right`.
3. Un indicador booleano de urgencia.

La orden lateral y el indicador de urgencia son señales intermedias utilizadas por el gestor de maniobras laterales.

### Salidas del gestor lateral

El gestor lateral devuelve:

1. Una nueva ruta cuando es necesario iniciar un adelantamiento o un retorno.
2. El estado actual de la máquina lateral.

Cuando no se requiere una actualización, el gestor devuelve `None` como ruta. El componente encargado de la integración debe conservar y utilizar la última ruta válida.

Las salidas finales utilizadas por los componentes de control son la velocidad de referencia y la ruta vigente.

## Flujo de ejecución

En cada ciclo de decisión se realizan las siguientes operaciones:

1. El observador procesa las representaciones del entorno.
2. El planificador principal obtiene las distancias y los indicadores necesarios.
3. Se selecciona la restricción activa.
4. La máquina longitudinal determina el estado de conducción.
5. Se calcula y filtra la velocidad de referencia.
6. El planificador evalúa las condiciones para iniciar una maniobra lateral.
7. El gestor lateral actualiza el estado del adelantamiento o del retorno.
8. Cuando corresponde, se genera una nueva ruta.
9. La velocidad de referencia y la ruta vigente se proporcionan a los componentes de control.

## Estructura del repositorio

```text
Urban-FSM-Decision-Making-Module/
├── README.md
├── src/
│   ├── README.md
│   ├── basic_planner.py
│   └── lane_change_manager.py
└── evaluation/
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

## Código fuente

### `src/basic_planner.py`

Este archivo contiene:

- `BasicPlannerConfig`: parámetros de configuración del observador y del planificador.
- `BasicEnvironmentObserver`: transformación de las entradas del entorno en distancias métricas e indicadores lógicos.
- `BasicPlanner`: selección de la restricción activa, actualización de la máquina longitudinal, cálculo de la velocidad de referencia y generación de solicitudes laterales.

### `src/lane_change_manager.py`

Este archivo contiene:

- La máquina de estados lateral.
- La generación de rutas desplazadas.
- La supervisión del progreso del adelantamiento.
- La comprobación de las condiciones de retorno.
- La generación de la transición hacia la ruta original.

### `src/README.md`

Proporciona una descripción técnica más detallada de las clases y de las interfaces incluidas en la carpeta `src`.

## Evaluación

La carpeta `evaluation` contiene los archivos utilizados para organizar y analizar la evaluación del módulo mediante 44 rutas seleccionadas de Bench2Drive.

### Rutas

La carpeta `evaluation/routes` contiene:

- `bench2drive44_selected.xml`: definición de las 44 rutas utilizadas en la evaluación.
- `bench2drive44_mapping.csv`: correspondencia entre los identificadores utilizados en el repositorio y las rutas originales de Bench2Drive.

### Scripts de procesamiento

La carpeta `evaluation/scripts` contiene:

- `merge_jsons_44.py`: combina los resultados individuales de las 44 ejecuciones en un único archivo.
- `b2d44_metrics_from_results_json.py`: procesa los resultados combinados y genera las métricas de evaluación en formato CSV.

Las instrucciones específicas para utilizar estos scripts se encuentran en `evaluation/README.md`.

### Resultados individuales

La carpeta `evaluation/results/individual` contiene los archivos JSON producidos por cada ejecución:

```text
results_00.json
results_01.json
...
results_43.json
```

Cada archivo conserva los resultados correspondientes a una de las 44 rutas seleccionadas.

### Resultados agregados

La carpeta `evaluation/results` contiene también:

| Archivo | Contenido |
|---|---|
| `merged_44_results.json` | Resultados combinados de las 44 rutas. |
| `merge_report.csv` | Informe generado durante la combinación de los archivos individuales. |
| `global_metrics_44.csv` | Métricas agregadas del conjunto completo. |
| `ability_metrics_44.csv` | Métricas agrupadas según las capacidades evaluadas. |
| `per_route_metrics_44.csv` | Métricas desglosadas para cada ruta. |

Esta organización mantiene la trazabilidad entre cada ejecución individual, la ruta correspondiente y los resultados agregados utilizados en el análisis.


## Integración

Para utilizar el módulo es necesario conectar las salidas de percepción, localización y planificación con las entradas esperadas por `BasicPlanner`.

El planificador principal produce la velocidad de referencia y la solicitud lateral. Esta solicitud se proporciona a `LaneChangeManager`, que actualiza su máquina de estados y genera una nueva ruta cuando la maniobra lo requiere.

El componente encargado de la integración debe:

- Ejecutar ambos componentes en el orden correspondiente.
- Conservar la última ruta válida cuando el gestor lateral devuelve `None`.
- Proporcionar la velocidad de referencia al controlador longitudinal.
- Proporcionar la ruta vigente al componente encargado del seguimiento lateral.


## Consideraciones sobre la percepción

La representación BEV utilizada por la arquitectura asigna la misma categoría semántica a peatones y bicicletas. Por este motivo, el módulo trata esta entrada como una categoría de actor vulnerable y aplica una frenada prioritaria cuando se detecta a corta distancia.

La distinción explícita entre peatones y bicicletas requeriría información semántica adicional procedente del sistema de percepción.

## Contexto académico

Este repositorio forma parte de un Trabajo Fin de Grado del Grado en Ingeniería en Electrónica y Automática Industrial de la Universidad de Alcalá.

El trabajo se desarrolla en colaboración con el grupo de investigación RobeSafe y utiliza CARLA y Bench2Drive como entorno de simulación y evaluación.
