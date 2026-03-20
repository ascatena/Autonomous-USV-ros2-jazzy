# USV Autónomo — ROS 2 (Jazzy)

> Sistema de navegación autónoma para un vehículo de superficie no tripulado (USV) tipo catamarán, desarrollado sobre ROS 2 Jazzy. Integra fusión de sensores (IMU + GPS), odometría sintética y la pila de navegación Nav2, adaptados a las restricciones cinemáticas y dinámicas de una embarcación de superficie. La operación remota y el monitoreo en tiempo real se realizan desde tierra mediante Foxglove Studio sobre una conexión WiFi.

---

## Tabla de Contenidos

- [Descripción General](#descripción-general)
- [Arquitectura del Sistema](#arquitectura-del-sistema)
- [Estructura del Workspace](#estructura-del-workspace)
- [Paquete robot\_1 — Interfaz de Hardware](#paquete-robot_1--interfaz-de-hardware)
- [Paquete robot\_config — Localización y Navegación](#paquete-robot_config--localización-y-navegación)
- [Instalación y Dependencias](#instalación-y-dependencias)
- [Uso](#uso)
- [Decisiones de Diseño y Limitaciones](#decisiones-de-diseño-y-limitaciones)
- [Trabajo Futuro](#trabajo-futuro)

---

## Descripción General

### El vehículo

El USV es un catamarán de construcción propia compuesto por dos cascos rígidos ahuecados, unidos entre sí por dos travesaños de aluminio. Cada casco aloja la electrónica de propulsión y está sellado de forma estanca mediante tapas superiores con o-rings que garantizan la impermeabilidad. La propulsión se logra mediante dos motores unidireccionales, uno ubicado en la popa de cada casco. La dirección se controla mediante un único timón gobernado por un servo, cuyo eje emerge de la caja estanca central.

Sobre los travesaños de aluminio se atornilla una caja estanca independiente que aloja la computadora a bordo (Raspberry Pi 5). Esta caja se comunica mediante cable LAN con el STM32, que actúa como controlador de bajo nivel gestionando los sensores (IMU MPU9250 y GPS) montados en PCB, así como los actuadores (motores y servo de timón).

```
        ┌──────────────────────────────────────────┐
        │          CAJA ESTANCA CENTRAL             │
        │              Raspberry Pi 5               │
        │         ROS 2 Jazzy | Nav2 | EKF          │
        └────────────────┬─────────────────────────┘
                         │ UART (cable LAN)
        ┌────────────────▼─────────────────────────┐
        │                 STM32                     │
        │   [IMU MPU9250]  [GPS]  [Servo timón]    │
        └──────────┬───────────────────┬───────────┘
                   │                   │
          [Motor babor]         [Motor estribor]
            Casco izq.            Casco der.
```

### Objetivo del sistema

El sistema permite al USV operar en dos modos:

- **Manual (Teleop):** Un operador envía comandos de velocidad en tiempo real desde la estación en tierra a través de WiFi, con prioridad absoluta sobre el sistema autónomo. El barco responde inmediatamente y Nav2 queda inhibido durante el tiempo que dure el control manual.
- **Autónomo:** El operador define waypoints geográficos (latitud/longitud) desde Foxglove Studio. Nav2 planifica y ejecuta la trayectoria utilizando un Filtro de Kalman Extendido (EKF) que fusiona datos de la IMU, el GPS y una odometría sintética para estimar la posición del barco en todo momento.

### Interfaz de usuario y monitoreo

La interacción con el USV desde tierra se realiza a través de **Foxglove Studio**, conectado al sistema ROS 2 a bordo mediante **`foxglove_bridge`** (protocolo WebSocket sobre WiFi). Esta herramienta permite:

- **Visualizar en tiempo real** los tópicos del sistema: posición GPS, orientación IMU, odometría, velocidades comandadas, estado de Nav2 y transformaciones TF.
- **Enviar waypoints** al sistema de navegación autónoma mediante el plugin de mapa integrado, haciendo clic directamente sobre el mapa para definir los puntos de destino.
- **Teleoperar el barco** publicando comandos de velocidad en `/turtle1/cmd_vel` desde paneles de control personalizados.
- **Monitorear el estado** de los filtros EKF, el árbol de comportamiento de Nav2 y los datos crudos de los sensores.

La conexión WiFi es el único canal de comunicación entre la estación en tierra y el USV durante la operación. Toda la inteligencia de navegación reside y se ejecuta a bordo en la Raspberry Pi 5.

### Stack tecnológico

| Componente | Tecnología |
|---|---|
| Sistema operativo | Linux (Ubuntu / Raspberry Pi OS de 64 bits) |
| Framework robótico | ROS 2 Jazzy |
| Computadora a bordo | Raspberry Pi 5 |
| Microcontrolador | STM32 |
| Sensores | GPS (NMEA), IMU MPU9250 (acelerómetro + giroscopio + magnetómetro) |
| Actuadores | 2× motor unidireccional + 1× servo de timón |
| Comunicación RPi 5 ↔ STM32 | UART sobre cable LAN |
| Comunicación tierra ↔ USV | WiFi (IEEE 802.11) |
| Interfaz de usuario | Foxglove Studio + `foxglove_bridge` (WebSocket) |
| Localización | `robot_localization` (doble EKF) |
| Transformación GPS | `navsat_transform_node` (`robot_localization`) |
| Navegación autónoma | Nav2 (Pure Pursuit Controller + NavFn Planner) |
| Build system | `colcon` |

---

## Arquitectura del Sistema

### Tres capas del sistema

El sistema se organiza en tres capas que interactúan entre sí:

**Capa de tierra (Estación de control):** Una PC con Foxglove Studio conectada por WiFi al USV. Desde aquí el operador visualiza el estado del sistema, teleopera el barco y define waypoints geográficos en el mapa.

**Capa de ROS 2 (Raspberry Pi 5 a bordo):** Es el núcleo del sistema. Recibe tramas UART del STM32 y las convierte en mensajes estándar de ROS 2. Fusiona datos de sensores con filtros EKF para estimar la posición. Ejecuta Nav2 para planificar y seguir rutas. Implementa un multiplexor que arbitra entre comandos manuales y autónomos, priorizando siempre al operador humano.

**Capa de hardware (STM32):** Adquiere datos crudos de la IMU y el GPS, los empaqueta y los envía por UART. Recibe comandos de velocidad y los traduce al control físico de motores y servo.

### Diagrama de comunicación global

![Figura 1. Diagrama de tópicos y Nodos implementados](imgs/USV_DiagramaNodosTopicos.drawio.png)   

### Mapa de tópicos principales

| Tópico | Tipo | Productor | Consumidor | Descripción |
|---|---|---|---|---|
| `/imu` | `sensor_msgs/Imu` | `uart_bridge` | `ekf_local`, `ekf_global`, `navsat_transform` | Aceleración, velocidad angular y orientación (IMU calibrada, frame: `imu_frame`) |
| `/mpu9250/mag` | `sensor_msgs/MagneticField` | `uart_bridge` | Foxglove *(monitoreo)* | Campo magnético crudo del MPU9250 (frame: `imu_frame`) |
| `/gps/fix` | `sensor_msgs/NavSatFix` | `uart_bridge` | `navsat_transform`, Foxglove *(monitoreo)* | Posición GPS (lat/lon/alt) con estado de fix (frame: `gps_frame`) |
| `/heading` | `std_msgs/Float32` | `uart_bridge` | Foxglove *(monitoreo)* | Rumbo magnético calculado desde el magnetómetro |
| `/wheel/odometry` | `nav_msgs/Odometry` | `synthetic_odom_publisher` | `ekf_local` | Odometría sintética con modelo de inercia (frame: `odom → base_link`) |
| `/odometry/gps` | `nav_msgs/Odometry` | `navsat_transform` | `ekf_global` | Posición GPS convertida a coordenadas cartesianas UTM (frame: `odom`) |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_global` | `bt_navigator`, `navsat_transform` ⚠️, Foxglove *(monitoreo)* | Estimación de posición fusionada en el mapa global. El ciclo hacia `navsat_transform` es intencional (doble EKF) |
| `/cmd_vel` | `geometry_msgs/Twist` | `controller_server` → `velocity_smoother` | MUX `uart_bridge` (secundario), `synthetic_odom_publisher` | Comandos de velocidad autónomos suavizados por `velocity_smoother` antes de llegar al MUX |
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | Foxglove Studio (WiFi) | MUX `uart_bridge` (prioritario) | Comandos de teleoperación manual — tienen prioridad absoluta sobre Nav2 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Foxglove Studio (WiFi) | `bt_navigator` | Waypoint destino definido desde el plugin de mapa de Foxglove |

### Transformaciones TF publicadas

| Transformación | Publicador | Frecuencia | Descripción |
|---|---|---|---|
| `map → odom` | `ekf_filter_node` | 20 Hz | Ancla la posición del robot al mapa global (GPS) |
| `odom → base_link` | `ekf_filter_node_local` | 30 Hz | Estimación continua del movimiento relativo del robot |
| `base_link → imu_frame` | `navsat.launch.py` (estática) | — | Offset físico de la IMU respecto al centro del barco |
| `base_link → gps_frame` | `navsat.launch.py` (estática) | — | Offset físico del GPS respecto al centro del barco |

### Configuración de la transformación GPS (`navsat.yaml`)

El nodo `navsat_transform_node` convierte las coordenadas geográficas del GPS (lat/lon WGS84) a coordenadas cartesianas planas (UTM) utilizables por ROS 2, publicando el resultado en `/odometry/gps` para que el EKF global pueda consumirlo. Además, se suscribe a `/odometry/filtered` (la salida del EKF global) para conocer la posición actual del robot y alinear correctamente cada nueva lectura GPS — esto crea un ciclo intencional entre `navsat_transform_node` y `ekf_global` que es la base de la arquitectura de doble EKF. Sus parámetros más relevantes son:

- **`use_odometry_yaw: true`** — El yaw de referencia para la transformación se toma de la odometría (EKF local) y no directamente de la IMU, lo que mejora la estabilidad ante perturbaciones del campo magnético.
- **`magnetic_declination_radians: -0.18006`** — Declinación magnética configurada para la zona de operación (aproximadamente La Plata, Argentina). Compensa la diferencia entre el norte magnético y el norte geográfico.
- **`zero_altitude: true`** — Fuerza la altitud a 0, adecuado para operación en superficie.
- **`broadcast_cartesian_transform: true`** — Publica la transformación TF cartesiana necesaria para que Nav2 ubique al robot en el mapa.
- **`wait_for_datum: false`** — El sistema no espera un datum fijo; se inicializa con la primera lectura GPS válida.
- **`frequency: 30 Hz`**, **`delay: 1.0 s`** — Opera a 30 Hz con una ventana de sincronización de 1 segundo para alinear temporalmente las lecturas de IMU, GPS y odometría.

---

## Estructura del Workspace

```
ros2_ws/
├── docs/
│   └── images/                         ← Imágenes y diagramas del proyecto
│       ├── usv_overview.jpg
│       ├── usv_casco_motor.jpg
│       ├── usv_electronica.jpg
│       ├── usv_rpi5.jpg
│       ├── diagram_hardware.png
│       └── diagram_ros2.png
└── src/
    ├── robot_1/                         ← Paquete de interfaz de hardware
    │   └── robot_1/
    │       ├── uart_bridge.py           ← Nodo principal de comunicación serie
    │       ├── estimacion_2_odom.py     ← Odometría sintética con inercia
    │       ├── cmd_vel_listener.py      ← Utilidad de debug (no activo en producción)
    │       ├── nav2_manager.py          ← Cliente de waypoints (en desarrollo)
    │       ├── estimacion_odom.py       ← Versión anterior de odometría (archivada)
    │       └── VIEJOuart_b_prom.py      ← Versión anterior de uart_bridge (archivada)
    └── robot_config/                    ← Paquete de localización y navegación
        ├── config/
        │   ├── ekf_local.yaml           ← Parámetros EKF local (odom→base_link)
        │   ├── ekf.yaml                 ← Parámetros EKF global (map→odom)
        │   ├── navsat.yaml              ← Parámetros transformación GPS→UTM
        │   ├── nav2_params.yaml         ← Parámetros completos de Nav2
        │   └── usv_nav.xml              ← Árbol de comportamiento Nav2
        └── launch/
            ├── launch_general.launch.py     ← Entry point principal del sistema
            ├── node_localization.launch.py  ← Orquestador de localización
            ├── navsat.launch.py             ← TFs estáticos + navsat_transform
            ├── ekf_local.launch.py          ← EKF local
            ├── ekf.launch.py                ← EKF global
            ├── navigation.launch.py         ← Stack completo de Nav2
            └── viejo_nav2.launch.py         ← Versión anterior de Nav2 (archivada)
```

---

### `robot_1` — Archivos funcionales

#### `uart_bridge.py`
Es el nodo más crítico del sistema. Cumple tres roles simultáneos: leer y parsear los datos crudos del hardware, publicarlos como mensajes estándar de ROS 2, y arbitrar qué comandos de velocidad se envían físicamente a los motores.

**Comunicación serie:** Lee continuamente el puerto `/dev/ttyAMA0` a 30 Hz. Cada trama entrante tiene 55 bytes y comienza con un byte de cabecera que indica su tipo: `0x49` ('I') para datos de IMU y `0x47` ('G') para datos de GPS. El struct de IMU se desempaqueta como `<9i4f` (9 enteros + 4 floats), obteniendo acelerómetro, giroscopio, magnetómetro y cuaternión. El struct de GPS extrae latitud, longitud, altitud, número de satélites y estado del fix.

**Calibración de IMU:** Durante los primeros 20 segundos de operación el nodo acumula lecturas de acelerómetro y giroscopio en un buffer, calcula el promedio de cada eje y lo guarda como offset. A partir de ese momento todas las publicaciones aplican estos offsets para eliminar el sesgo (bias) del sensor. Durante la calibración no se publican datos de IMU.

**Orientación:** El cuaternión que provee el MPU9250 se usa para roll y pitch, pero el yaw se reemplaza completamente por el heading calculado desde el magnetómetro (con offsets hardcodeados: `mx-120.2`, `my-118.3`, `mz+67.1`). Esto se hace porque el yaw del MPU9250 acumula deriva, mientras que el magnetómetro provee una referencia absoluta al norte magnético.

**MUX de control:** Implementa un sistema de prioridad entre dos fuentes de comandos. El canal manual (`/turtle1/cmd_vel`) tiene prioridad absoluta: cada vez que llega un mensaje manual se actualiza un timestamp. El canal autónomo (`/cmd_vel` de Nav2) solo se procesa si han pasado más de `teleop_timeout` segundos (1.0 s por defecto) desde el último comando manual. Incluye una lógica especial para giros estáticos: si se detecta `angular.z ≠ 0` con `linear.x ≈ 0`, fuerza `linear.x = 1.0` para asegurar flujo de agua sobre el timón (necesario para que el timón tenga efecto). Los comandos se empaquetan como `'V' + pack('<BB', linear_x, angular_z)` donde `linear_x ∈ [0,255]` y `angular_z ∈ [0,255]` centrado en 128.

**Watchdog serial:** Cada 2 segundos envía la trama `b"W\1\1"` al STM32 como señal de vida. Si el STM32 deja de recibir esta trama, puede implementar una rutina de seguridad (parada de motores) del lado del firmware.

---

#### `estimacion_2_odom.py`
Resuelve el problema de la ausencia de encoders en los motores del barco. Sin encoders no hay forma de medir directamente cuánto se desplazó el barco, por lo que este nodo genera una odometría "sintética" a partir de los comandos de velocidad y un modelo simplificado de inercia acuática.

**Modelo de inercia:** En lugar de aplicar los comandos de velocidad instantáneamente, aplica un filtro de primer orden: `velocidad_actual += (velocidad_deseada - velocidad_actual) × 0.1`. Este factor (0.1) simula la inercia del barco en el agua, suavizando las transiciones de velocidad. Cuando la velocidad actual cae por debajo de 0.01 m/s se fuerza a cero (snap-to-zero) para evitar deriva infinitesimal.

**Integración de posición:** En cada ciclo de 30 Hz integra la velocidad actual en el tiempo para actualizar la posición teórica `(x, y, θ)` del barco. Esta posición no representa la posición real en el mundo (para eso está el GPS), sino el desplazamiento relativo desde el inicio, que el EKF local usa para estimar la transformación `odom → base_link`.

**Watchdog:** Si no recibe ningún mensaje en `/cmd_vel` durante más de 1 segundo, fuerza los targets de velocidad a cero, lo que hace que el barco frene gradualmente por el modelo de inercia. Esto evita que el barco continúe "navegando virtualmente" si Nav2 deja de publicar comandos.

**Covarianza dinámica:** Cuando el barco está detenido (`target_vx ≈ 0` y `target_wz ≈ 0`) la covarianza del twist se establece en `1e-9` (confianza casi absoluta en que la velocidad es cero), lo que actúa como un "ancla" para el EKF y evita que el filtro derive durante las paradas. Cuando el barco está en movimiento la covarianza sube a `0.5` (baja confianza), indicando al EKF que priorice otros sensores. La covarianza de pose se mantiene fija en `0.1` ya que la posición absoluta no se usa para localizar al robot.

---

#### `cmd_vel_listener.py`
Nodo de utilidad para depuración. Se suscribe a `/turtle1/cmd_vel` y loguea por consola los valores de `linear.x` y `angular.z` cada vez que llega un mensaje. No publica ningún tópico ni tiene ningún rol en el sistema productivo. Es útil durante el desarrollo para verificar que los comandos de teleoperación llegan correctamente antes de conectar el hardware.

---

#### `nav2_manager.py`
Cliente de acción para el envío programático de waypoints a Nav2. Implementa un `ActionClient` del tipo `FollowWaypoints` que construye una lista de `PoseStamped` y la envía al action server de Nav2. Incluye callbacks de feedback (waypoint actual), respuesta de goal (aceptado/rechazado) y resultado final.

**Estado actual:** El nodo está en desarrollo. El `main()` contiene tres waypoints hardcodeados en formato `(lat, lon)` correspondientes a una zona de prueba, pero la conversión de coordenadas GPS a coordenadas del marco `map` aún no está implementada. En el sistema activo los waypoints se envían desde Foxglove Studio vía `/goal_pose`. Este nodo está pensado para misiones autónomas preprogramadas sin intervención del operador.

---

### `robot_config` — Archivos de configuración

#### `ekf_local.yaml`
Configura el EKF local (`ekf_filter_node_local`) que opera a 30 Hz con `world_frame: odom`. Su función es proporcionar una estimación continua y suave del movimiento relativo del barco, publicando la transformación `odom → base_link`.

Fusiona dos fuentes de datos: la odometría sintética de `/wheel/odometry` (se usan las velocidades lineales vx, vy, vz y la velocidad angular wz) y la IMU de `/imu` (se usan orientación en roll/pitch/yaw, velocidades angulares y aceleraciones lineales). No consume GPS, por lo que su estimación deriva con el tiempo pero es muy estable en el corto plazo. La `process_noise_covariance` está configurada con valores bajos en posición (`1e-3`) y valores moderados en velocidades (`0.5`), reflejando que el modelo cinemático del barco no es perfectamente conocido.

---

#### `ekf.yaml`
Configura el EKF global (`ekf_filter_node`) que opera a 20 Hz con `world_frame: map`. Su función es anclar la posición del barco al mundo real usando el GPS, publicando la transformación `map → odom` y el tópico `/odometry/filtered`.

Fusiona la IMU de `/imu` (orientación y velocidades angulares) y la posición GPS cartesiana de `/odometry/gps` (x, y, z absoluto proveniente de `navsat_transform_node`). La odometría de `/wheel/odometry` está comentada en esta configuración, dejando que el GPS sea la fuente primaria de posición absoluta. La `process_noise_covariance` es más permisiva en posición (`1.0`) que en el EKF local, aceptando que la posición GPS tiene mayor incertidumbre natural.

---

#### `navsat.yaml`
Configura el nodo `navsat_transform_node`, cuya función principal es convertir las coordenadas geográficas del GPS (latitud/longitud WGS84) a coordenadas cartesianas planas (UTM) utilizables por ROS 2.

Los parámetros más relevantes son: `magnetic_declination_radians: -0.18006` (declinación magnética para la zona de La Plata, Argentina, obtenida de NOAA), `use_odometry_yaw: true` (el yaw para alinear la transformación GPS→UTM se toma de la odometría vía TF `odom→base_link`, no de la IMU directamente), `zero_altitude: true` (fuerza altitud a 0 para operación en superficie), `broadcast_cartesian_transform: true` (publica la TF cartesiana necesaria para Nav2) y `wait_for_datum: false` (se inicializa con la primera lectura GPS válida, sin esperar un datum fijo). Opera a 30 Hz con una ventana de sincronización de 1 segundo para alinear temporalmente las lecturas de IMU, GPS y odometría.

---

#### `nav2_params.yaml`
Archivo central de configuración de toda la pila Nav2, adaptado específicamente a las restricciones cinemáticas y dinámicas de una embarcación de superficie.

**Controller (Pure Pursuit):** Se eligió `RegulatedPurePursuitController` en lugar del controlador DWB porque Pure Pursuit es más adecuado para vehículos no holonómicos con alta inercia. `use_rotate_to_heading: false` desactiva la rotación en el lugar (imposible para un barco con timón). `allow_reversing: false` desactiva la marcha atrás (los motores son unidireccionales). `desired_linear_vel: 1.0 m/s` con `lookahead_dist: 1.5 m`.

**Goal checker:** `xy_goal_tolerance: 4.0 m` y `yaw_goal_tolerance: 0.5 rad`. La tolerancia de 4 metros reconoce el error natural del GPS y la inercia del barco al detenerse.

**Costmaps:** Local (40×40 m) y global (100×100 m, rolling window) configurados sin capa de obstáculos estáticos, solo con `inflation_layer`. Esto es intencional: el barco opera en cuerpos de agua abiertos sin obstáculos mapeados. `resolution: 0.5 m`.

**Planner (NavFn):** `allow_unknown: true` permite planificar sobre celdas desconocidas del costmap, esencial para navegación en mar abierto donde no hay mapa previo. `tolerance: 4.0 m` es consistente con la tolerancia del goal checker.

---

#### `usv_nav.xml`
Define el árbol de comportamiento (Behavior Tree) que Nav2 ejecuta para navegar hacia un waypoint. Está simplificado respecto al árbol por defecto de Nav2 para adaptarse a la operación en entornos acuáticos abiertos donde los comportamientos de recuperación complejos (como retroceder o girar en el lugar) no son aplicables.

El árbol intenta computar un camino hacia el goal y seguirlo. Si la navegación falla, ejecuta una única acción de recuperación: esperar 5 segundos (`<Wait wait_duration="5.0"/>`) antes de reintentar. Esta estrategia de recuperación minimalista es deliberada: en un barco, las maniobras de recuperación agresivas (spin, backup) pueden ser contraproducentes o imposibles dadas las restricciones cinemáticas.

---

### `robot_config` — Launch files

#### `launch_general.launch.py`
Es el entry point del sistema completo. Orquesta el arranque de todos los subsistemas en el orden correcto usando un `TimerAction` de 5 segundos entre la localización y Nav2. Este retardo es fundamental: permite que los filtros EKF se inicialicen y estabilicen con las primeras lecturas de IMU y GPS antes de que Nav2 intente leer las transformaciones TF. Si Nav2 arrancara inmediatamente, fallaría al no encontrar las TFs `map→odom→base_link` disponibles.

Secuencia de arranque:
1. `navsat.launch.py` — TFs estáticos + navsat_transform_node
2. `node_localization.launch.py` — EKF local + EKF global
3. *(espera 5 segundos)*
4. `navigation.launch.py` — Stack completo de Nav2

---

#### `node_localization.launch.py`
Orquestador de la capa de localización. Lanza en orden: `navsat.launch.py`, `ekf_local.launch.py` y `ekf.launch.py`. Centraliza el arranque de los tres nodos de localización para que puedan ser iniciados como una unidad tanto desde `launch_general.launch.py` como de forma independiente durante el desarrollo o depuración.

---

#### `navsat.launch.py`
Lanza el nodo `navsat_transform_node` con la configuración de `navsat.yaml` y define las **transformaciones TF estáticas** que describen la posición física de los sensores respecto al centro del barco (`base_link`): la IMU en `imu_frame` y el GPS en `gps_frame`. Estas TFs estáticas son fundamentales para que el EKF pueda compensar el offset físico entre los sensores y el punto de referencia del robot.

---

#### `navigation.launch.py`
Lanza todos los nodos del stack Nav2: `controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother` y `lifecycle_manager`. Resuelve en Python la ruta absoluta del archivo XML del árbol de comportamiento (`usv_nav.xml`) y la inyecta directamente como parámetro del `bt_navigator`, sobreescribiendo el placeholder `{path_to_pkg}` del YAML. Todos los nodos se gestionan bajo el ciclo de vida de ROS 2 a través del `lifecycle_manager`.

---