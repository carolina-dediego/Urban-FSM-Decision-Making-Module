# Código fuente

Esta carpeta contiene los componentes del módulo de toma de decisiones propuesto. Estos componentes se integran en la arquitectura modular de navegación autónoma de RobeSafe y no constituyen por sí solos un agente ejecutable de forma independiente.

## Componentes

### `basic_planner.py`

Este archivo contiene las siguientes clases:

* `BasicPlannerConfig`: agrupa los parámetros de configuración utilizados por el observador y el planificador.
* `BasicEnvironmentObserver`: transforma las representaciones BEV y la información de la infraestructura en distancias métricas e indicadores lógicos.
* `BasicPlanner`: selecciona la restricción activa, actualiza la máquina de estados longitudinal, calcula la velocidad de referencia y genera las solicitudes de maniobra lateral.

El planificador principal devuelve:

1. La velocidad de referencia, expresada en metros por segundo.
2. La orden lateral, representada mediante `keep`, `left` o `right`.
3. El indicador de urgencia asociado a la solicitud de cambio de carril.

### `lane_change_manager.py`

Este archivo contiene el gestor de maniobras laterales y la máquina de estados encargada de controlar el adelantamiento y el retorno al carril original.

La máquina lateral utiliza los siguientes estados:

* `GLOBAL`: el vehículo sigue la ruta de referencia.
* `OVERTAKE`: se genera y mantiene la ruta correspondiente al adelantamiento.
* `RETURN`: se genera la transición de retorno hacia la ruta original.

El gestor devuelve una ruta cuando es necesario actualizar la trayectoria seguida por el vehículo. Cuando no se genera una modificación, devuelve `None` y el componente que realiza la integración conserva la última ruta válida.

## Flujo de ejecución

En cada ciclo de decisión se realizan las siguientes operaciones:

1. El observador procesa las representaciones del entorno.
2. El planificador principal selecciona la restricción activa.
3. La máquina longitudinal determina el estado de conducción y la velocidad de referencia.
4. Cuando se cumplen las condiciones establecidas, el planificador genera una solicitud de maniobra lateral.
5. El gestor lateral actualiza el estado de la maniobra y, cuando corresponde, genera una nueva ruta de adelantamiento o de retorno.
