"""
Reto Final — Solo camara Wrist RGB (3).
Detecta el objeto naranja por color HSV, alinea visualmente
con servo proporcional y lo agarra.

Fases:
  align → centra el objeto en la camara con wrist_yaw / wrist_pitch
  reach → extiende el brazo hasta ~12 cm del objeto
  grab  → cierra gripper y sube el lift

Tecla 'r' para reiniciar a la fase align.
Ctrl+C para salir.
"""
from stretch_toolkit import (
    controller, teleop, merge_proportional, BACKEND_NAME,
    WRIST_RGB_CAMERA, WRIST_CAMERA, StateController,
)
import time
import cv2
import numpy as np

print(f"\n=== Running on {BACKEND_NAME} backend ===\n")

MIN_AREA = 200

# Perfiles de color HSV por objeto
OBJECTS = {
    "1": {
        "name": "Taza (roja)",
        "ranges": [
            (np.array([0,   100, 80]), np.array([10,  255, 255])),
            (np.array([170, 100, 80]), np.array([180, 255, 255])),
        ],
    },
    "2": {
        "name": "Pastillas (naranja)",
        "ranges": [
            (np.array([5, 150, 100]), np.array([20, 255, 255])),
        ],
    },
    "3": {
        "name": "Kleenex (negro)",
        "ranges": [
            (np.array([0, 0, 0]), np.array([180, 255, 60])),
        ],
    },
}

# Ganancias proporcionales
Kp_yaw   = 0.8
Kp_pitch = 0.8
Kp_arm   = 4.0

GRAB_DIST = 0.05   # metros — distancia objetivo para agarrar
DIST_TOL  = 0.08   # tolerancia para considerar "llegado"
REACH_TIMEOUT = 5.0  # segundos máximos en fase reach antes de agarrar igual
ALIGN_TOL = 0.08   # tolerancia de error normalizado para pasar a reach

GRAB_DURATION    = 2.0   # segundos cerrando garra
SHIFT_TURN_T     = 0.8   # segundos girando 90° para el desplazamiento lateral (ajustar)
SHIFT_MOVE_T     = 1.0   # segundos avanzando lateralmente (ajustar)
SIDE_TURN_T      = 0.25  # segundos para pequeño giro de "paso lateral" (ajustar)
SIDE_MOVE_T      = 0.6   # segundos para avanzar durante el "paso lateral" (ajustar)
ROTATE_DURATION  = 4.5   # segundos girando hasta la bandeja (ajustar)
ADVANCE_DURATION = 4.5   # segundos avanzando hacia la bandeja (ajustar)


def find_object(frame, color_ranges):
    """Devuelve el centroide del objeto más grande según los rangos HSV dados."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in color_ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]
    if not valid:
        return None, mask
    largest = max(valid, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None, mask
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy), mask


def main():
    print("Selecciona el objeto a agarrar:")
    for key, obj in OBJECTS.items():
        print(f"  {key} → {obj['name']}")
    choice = input("Opción (1/2/3): ").strip()
    while choice not in OBJECTS:
        choice = input("Opción inválida. Elige 1, 2 o 3: ").strip()
    color_ranges = OBJECTS[choice]["ranges"]
    print(f"\nObjetivo: {OBJECTS[choice]['name']}")
    print("Tecla 'r' = reiniciar fase. Ctrl+C = salir.\n")

    # Fase align: brazo recogido, gripper semiabierto.
    # No incluye wrist_yaw/pitch — el servo visual los controla.
    stow = StateController(controller, {
        "wrist_roll_counterclockwise": 0.0,
        "gripper_open": 0.4,
        "arm_out": 0.0,
    })

    # Fase reach: solo mantiene wrist neutral, NO controla arm_out ni gripper
    wrist_neutral = StateController(controller, {
        "wrist_roll_counterclockwise": 0.0,
    })

    # Fase place: baja el lift hasta el fondo (posición 0)
    lift_down = StateController(controller, {
        "lift_up": 0.0,
    })

    phase = "align"
    reach_ok = 0
    reach_start = None
    grab_start    = None
    shift_start   = None
    rotate_start  = None
    pause_start   = None
    advance_start = None
    print(f"Fase: {phase}")

    try:
        while True:
            velocities = teleop.get_normalized_velocities()
            auto_vel = {}

            frame = WRIST_RGB_CAMERA.get_frame()
            if frame is not None:
                centroid, mask = find_object(frame, color_ranges)
                display = frame.copy()
                h, w = frame.shape[:2]

                # Dibuja cruz en el centro
                cv2.line(display, (w // 2 - 15, h // 2), (w // 2 + 15, h // 2), (200, 200, 200), 1)
                cv2.line(display, (w // 2, h // 2 - 15), (w // 2, h // 2 + 15), (200, 200, 200), 1)

                if centroid is not None:
                    cx, cy = centroid
                    error_x = (cx - w / 2) / w   # normalizado -0.5 .. +0.5
                    error_y = (cy - h / 2) / h

                    cv2.circle(display, (cx, cy), 10, (0, 165, 255), -1)
                    cv2.line(display, (w // 2, h // 2), (cx, cy), (0, 255, 255), 2)

                    if phase == "align":
                        auto_vel["wrist_yaw_counterclockwise"] = -Kp_yaw * error_x
                        auto_vel["wrist_pitch_up"] = -Kp_pitch * error_y

                        if abs(error_x) < ALIGN_TOL and abs(error_y) < ALIGN_TOL and stow.is_at_goal():
                            phase = "reach"
                            reach_start = time.time()
                            print(f"\nFase: {phase}")

                    elif phase == "reach":
                        # Mantener centrado mientras extiende
                        auto_vel["wrist_yaw_counterclockwise"] = -Kp_yaw * error_x
                        auto_vel["wrist_pitch_up"] = -Kp_pitch * error_y

                        distance = WRIST_CAMERA.get_depth((cx, cy))
                        if distance is not None:
                            dist_err = distance - GRAB_DIST
                            auto_vel["arm_out"]  = Kp_arm * dist_err
                            auto_vel["lift_up"]  = -0.3   # bajar mientras se acerca
                            print(f"\rDist: {distance:.3f}m  Err: {dist_err:+.3f}m  ok:{reach_ok}   ", end="", flush=True)
                            if abs(dist_err) < DIST_TOL:
                                reach_ok += 1
                            else:
                                reach_ok = max(0, reach_ok - 1)  # decaer suave, no reseteo brusco
                        else:
                            # Sin lectura: avanzar despacio, mantener contador
                            auto_vel["arm_out"] = 0.3

                        timed_out = (reach_start is not None and
                                     time.time() - reach_start > REACH_TIMEOUT)
                        if reach_ok >= 5 or timed_out:
                            phase = "grab"
                            reach_ok = 0
                            print(f"\nFase: {phase}{'  [timeout]' if timed_out else ''}")

                cv2.putText(display, f"Fase: {phase}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow("Wrist RGB", display)
                cv2.imshow("Mascara Naranja", mask)

            # Aplicar velocidades segun fase
            if phase == "align":
                velocities = merge_proportional(velocities, auto_vel)
                velocities = merge_proportional(velocities, stow.get_command())
            elif phase == "reach":
                velocities = merge_proportional(velocities, auto_vel)
                velocities = merge_proportional(velocities, wrist_neutral.get_command())
            elif phase == "grab":
                if grab_start is None:
                    grab_start = time.time()
                auto_vel["arm_out"]      = 0.5
                auto_vel["gripper_open"] = -1.0
                auto_vel["lift_up"]      = 0.4
                velocities = merge_proportional(velocities, auto_vel)
                if time.time() - grab_start > GRAB_DURATION:
                    phase = "shift"
                    shift_start = time.time()
                    print(f"\nFase: {phase}")

            elif phase == "shift":
                elapsed = time.time() - shift_start if shift_start else 0
                auto_vel["gripper_open"] = -1.0
                t0 = SHIFT_TURN_T
                t1 = t0 + SHIFT_MOVE_T

                if elapsed < t0:
                    auto_vel["base_counterclockwise"] = 0.6
                elif elapsed < t1:
                    auto_vel["base_forward"] = 0.5
                else:
                    phase = "rotate"
                    rotate_start = time.time()
                    print(f"\nFase: {phase}")

                velocities = merge_proportional(velocities, auto_vel)

            elif phase == "rotate":
                auto_vel["base_counterclockwise"] = 0.6
                auto_vel["gripper_open"]          = -1.0
                velocities = merge_proportional(velocities, auto_vel)
                if rotate_start is not None and time.time() - rotate_start > ROTATE_DURATION:
                    phase = "pause"
                    pause_start = time.time()
                    print(f"\nFase: {phase}")

            elif phase == "pause":
                auto_vel["gripper_open"] = -1.0
                velocities = merge_proportional(velocities, auto_vel)
                if pause_start is not None and time.time() - pause_start > 1.0:
                    phase = "advance"
                    advance_start = time.time()
                    print(f"\nFase: {phase}")

            elif phase == "advance":
                auto_vel["base_forward"] = 0.4
                auto_vel["arm_out"]      = 0.5
                auto_vel["gripper_open"] = -1.0
                velocities = merge_proportional(velocities, auto_vel)
                if advance_start is not None and time.time() - advance_start > ADVANCE_DURATION:
                    phase = "place"
                    print(f"\nFase: {phase}")

            elif phase == "place":
                # Bajar hasta el fondo; abrir garra solo cuando llegue
                if lift_down.is_at_goal():
                    auto_vel["gripper_open"] = 1.0
                else:
                    auto_vel["gripper_open"] = -1.0
                velocities = merge_proportional(velocities, auto_vel)
                velocities = merge_proportional(velocities, lift_down.get_command())

            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                phase = "align"
                reach_ok = 0
                reach_start = None
                grab_start    = None
                shift_start   = None
                rotate_start  = None
                pause_start   = None
                advance_start = None
                print(f"\nReiniciando → Fase: {phase}")

            controller.set_velocities(velocities)
            time.sleep(1 / 30)

    except KeyboardInterrupt:
        print("\n\nDeteniendo...")
    finally:
        controller.set_velocities({})
        controller.stop()
        cv2.destroyAllWindows()
        print("Listo.")


if __name__ == "__main__":
    main()