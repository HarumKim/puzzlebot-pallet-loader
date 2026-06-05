#!/usr/bin/env python3
"""
transform_obstacles_to_new_origin.py

Muestra el mapa SLAM con un click para marcar el nuevo origen del robot.
Transforma todos los obstáculos existentes al nuevo sistema de coordenadas
y genera el yaml de navegación listo para usar.

Uso:
    python3 src/slam/transform_obstacles_to_new_origin.py

Controles:
    click izquierdo  -> marcar nuevo origen (tu punto de inicio)
    u                -> deshacer el click
    s                -> guardar y salir
    q                -> salir sin guardar
"""

from pathlib import Path

import cv2
import numpy as np
import yaml


CALIBRATION_YAML = Path("src/slam/config/map_similarity_calibration.yaml")
OBSTACLES_YAML   = Path("src/slam/config/manual_obstacles.yaml")
CROP_IMAGE       = Path("src/slam/images/map_similarity_crop.png")
OUTPUT_YAML      = Path("src/puzzlebot_pallet_loader/config/real_robot_nav_params.yaml")

# Waypoint destino en el sistema de coordenadas del robot (metros desde tu inicio)
# Cambia estos valores si quieres un waypoint distinto
WAYPOINT_X = 1.00
WAYPOINT_Y = -1.60


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def pixel_to_slam(px, py, scale, crop_x_min, crop_y_max):
    return crop_x_min + px / scale, crop_y_max - py / scale


def transform_obstacles(obstacles_flat, tx, ty):
    """Aplica translación a lista plana [xmin,ymin,xmax,ymax, ...] en SLAM frame."""
    result = []
    for i in range(0, len(obstacles_flat), 4):
        xmin = obstacles_flat[i]     - tx
        ymin = obstacles_flat[i + 1] - ty
        xmax = obstacles_flat[i + 2] - tx
        ymax = obstacles_flat[i + 3] - ty
        result.extend([round(xmin, 3), round(ymin, 3),
                        round(xmax, 3), round(ymax, 3)])
    return result


def draw_obstacles_slam(img, obstacles_flat, scale, crop_x_min, crop_y_max):
    for i in range(0, len(obstacles_flat), 4):
        x1 = int((obstacles_flat[i]     - crop_x_min) * scale)
        y1 = int((crop_y_max - obstacles_flat[i + 1]) * scale)
        x2 = int((obstacles_flat[i + 2] - crop_x_min) * scale)
        y2 = int((crop_y_max - obstacles_flat[i + 3]) * scale)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 200, 0), 1)


def main():
    calib = load_yaml(CALIBRATION_YAML)
    obs_data = load_yaml(OBSTACLES_YAML)

    scale       = float(calib["basis"]["scale_px_per_m"])
    crop_x_min  = float(calib["crop"]["world_bounds_m"]["x_min"])
    crop_y_max  = float(calib["crop"]["world_bounds_m"]["y_max"])

    obstacles_flat = [float(v) for v in obs_data["obstacles_flat_for_astar"]]

    base_img = cv2.imread(str(CROP_IMAGE))
    if base_img is None:
        raise FileNotFoundError(f"No se encontró: {CROP_IMAGE}")

    origin_px = None

    def on_click(event, x, y, flags, param):
        nonlocal origin_px
        if event == cv2.EVENT_LBUTTONDOWN:
            origin_px = (x, y)

    cv2.namedWindow("Marca tu punto de inicio", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Marca tu punto de inicio", 900, 900)
    cv2.setMouseCallback("Marca tu punto de inicio", on_click)

    print("\n=== Transform Obstacles ===")
    print("CLICK en tu punto de inicio en el mapa.")
    print("  u -> deshacer   s -> guardar   q -> salir\n")

    while True:
        display = base_img.copy()

        # Dibuja obstáculos en SLAM frame (verde)
        draw_obstacles_slam(display, obstacles_flat, scale, crop_x_min, crop_y_max)

        if origin_px is not None:
            tx, ty = pixel_to_slam(origin_px[0], origin_px[1], scale,
                                   crop_x_min, crop_y_max)
            # Cruz en el origen marcado
            cv2.drawMarker(display, origin_px, (0, 0, 255),
                           cv2.MARKER_CROSS, 20, 2)
            cv2.putText(display,
                        f"Inicio: ({tx:.2f}, {ty:.2f}) SLAM",
                        (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 255), 2)

            # Preview de obstáculos transformados (amarillo)
            new_obs = transform_obstacles(obstacles_flat, tx, ty)
            for i in range(0, len(new_obs), 4):
                x1 = int((new_obs[i]     + tx - crop_x_min) * scale)
                y1 = int((crop_y_max - (new_obs[i + 1] + ty)) * scale)
                x2 = int((new_obs[i + 2] + tx - crop_x_min) * scale)
                y2 = int((crop_y_max - (new_obs[i + 3] + ty)) * scale)
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 220, 220), 1)
        else:
            cv2.putText(display,
                        "Click para marcar tu punto de inicio",
                        (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (200, 200, 0), 2)

        cv2.putText(display, "Verde=SLAM  Amarillo=tu frame  |  s=guardar  q=salir",
                    (10, display.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow("Marca tu punto de inicio", display)
        key = cv2.waitKey(20) & 0xFF

        if key == ord('u'):
            origin_px = None
            print("Origen borrado.")
        elif key == ord('s'):
            if origin_px is None:
                print("Marca tu punto de inicio primero.")
                continue
            break
        elif key == ord('q'):
            cv2.destroyAllWindows()
            print("Saliendo sin guardar.")
            return

    cv2.destroyAllWindows()

    tx, ty = pixel_to_slam(origin_px[0], origin_px[1], scale, crop_x_min, crop_y_max)
    new_obs = transform_obstacles(obstacles_flat, tx, ty)

    print(f"\nOrigen marcado en SLAM: ({tx:.3f}, {ty:.3f})")
    print(f"Obstáculos transformados a tu frame ({len(new_obs)//4} rectángulos):")
    for i in range(0, len(new_obs), 4):
        print(f"  [{new_obs[i]:.3f}, {new_obs[i+1]:.3f}, "
              f"{new_obs[i+2]:.3f}, {new_obs[i+3]:.3f}]")

    # Bounds: área que cubre todos los obstáculos + margen
    all_x = [new_obs[i] for i in range(0, len(new_obs), 4)] + \
            [new_obs[i] for i in range(2, len(new_obs), 4)]
    all_y = [new_obs[i] for i in range(1, len(new_obs), 4)] + \
            [new_obs[i] for i in range(3, len(new_obs), 4)]
    margin = 0.3
    bounds = [round(min(all_x) - margin, 2), round(min(all_y) - margin, 2),
              round(max(all_x) + margin, 2), round(max(all_y) + margin, 2)]

    output = {
        "hybrid_a_star_bug0_node": {
            "ros__parameters": {
                "waypoints": [WAYPOINT_X, WAYPOINT_Y],
                "goal_tolerance": 0.20,
                "bounds": bounds,
                "resolution": 0.08,
                "inflation_radius": 0.20,
                "path_stride": 2,
                "known_obstacle_tolerance": 0.20,
                "known_obstacle_angle_window_deg": 20.0,
                "obstacles": new_obs,
            }
        }
    }

    OUTPUT_YAML.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_YAML, "w") as f:
        yaml.dump(output, f, sort_keys=False, default_flow_style=False)

    print(f"\nGuardado en: {OUTPUT_YAML}")
    print(f"Waypoint configurado: ({WAYPOINT_X}, {WAYPOINT_Y})")
    print(f"Bounds calculados: {bounds}")


if __name__ == "__main__":
    main()
