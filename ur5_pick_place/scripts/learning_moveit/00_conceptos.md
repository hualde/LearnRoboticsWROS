# Conceptos Básicos de MoveIt 2

Para entender cómo usar MoveIt desde Python, debemos conocer estos pilares:

### 1. El `Move Group`
Es el concepto más importante. Un robot se divide en "grupos de planificación". 
*   En este proyecto tenemos `ur5_manipulator` (el brazo) y `robotiq_gripper` (la pinza).
*   Cada movimiento se solicita a un grupo específico.

### 2. Planning vs Execution
MoveIt no solo "mueve" el robot. El proceso es:
1.  **Planning (Planificación):** Calcula una trayectoria desde el punto A al B evitando obstáculos.
2.  **Execution (Ejecución):** Envía esa trayectoria a los controladores de los motores.
En nuestros scripts usaremos un comando que hace ambas cosas a la vez para simplificar.

### 3. Tipos de Objetivos (Goals)
Podemos decirle al robot qué hacer de tres formas:
*   **Named Target:** "Ve a la posición 'home'" (definida previamente en el SRDF).
*   **Joint-space Target:** "Mueve el motor 1 a 45º, el motor 2 a -10º..."
*   **Pose Target:** "Lleva la pinza a la coordenada X=0.5, Y=0.2, Z=1.1".

### 5. Planificadores Industriales (Pilz)
No todos los planes son iguales. En este proyecto usamos **Pilz Industrial Motion Planner**:
*   **PTP (Point-To-Point)**: El robot se mueve de forma eficiente buscando el camino más fluido en el espacio de juntas. Puede hacer curvas.
*   **LIN (Linear)**: Obliga al robot a trazar una línea recta perfecta en el espacio cartesiano. Útil para movimientos de precisión.
*   **Requisito Crítico**: Estos planificadores fallan si los joints no tienen definidos límites de aceleración en `joint_limits.yaml`.

---

## 🛠️ Guía de Scripts en esta Carpeta

Para tu aprendizaje, hemos creado esta secuencia de scripts:

1.  **`01_intro_action.py`**: El primer paso. Aprenderás cómo conectar tu código Python al "cerebro" de MoveIt.
2.  **`02_move_joints.py`**: Control directo. Mueve cada motor del brazo por separado usando ángulos (grados).
3.  **`03_move_cartesian.py`**: El poder de la IA. Tú le das una coordenada X, Y, Z y el robot calcula cómo llegar.
4.  **`04_get_current_pose.py`**: Tu herramienta de diagnóstico. Te dice exactamente dónde está la punta del robot en cada momento.
5.  **`05_gripper_control.py`**: El final del brazo. Aprende a abrir y cerrar la pinza Robotiq para agarrar objetos.
6.  **`06_sequential_pick_place.py`**: El "gran final" inicial. Una secuencia de 10 pasos para mover un objeto.
7.  **`07_planning_scene.py`**: Gestión de obstáculos. Añade muros y mesas para que el robot no choque.
8.  **`08_pick_place_refined.py`**: Versión **ROBUSTA**. Usa Pilz (PTP/LIN) y "Waypoints" para evitar colisiones complejas paso a paso.
9.  **`09_pick_place_automated.py`**: Versión **FINAL**. El ciclo completo automatizado sin esperas, optimizado para producción.

> [!TIP]
> Si MoveIt te devuelve un error `-31`, significa que la posición es físicamente inalcanzable (fuera de rango). Si devuelve `-2`, ¡cuidado! Es posible que estés a punto de chocar con un muro de la escena.
