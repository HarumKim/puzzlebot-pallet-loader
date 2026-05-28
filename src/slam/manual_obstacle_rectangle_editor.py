#!/usr/bin/env python3
"""
manual_obstacle_rectangle_editor.py

Editor visual para marcar obstáculos rectangulares manualmente sobre un mapa SLAM
ya recortado/alineado con map_similarity_calibration_click.py.

Objetivo:
- Dibujar rectángulos de obstáculos dando 2 clicks por rectángulo.
- Dibujar rectángulos de zonas libres/ignore/openings para documentar salidas tipo estacionamiento.
- Convertir cada rectángulo de pixeles a coordenadas reales en metros.
- Guardar YAML compatible con el nodo A* actual:
    obstacles: [xmin, ymin, xmax, ymax, ...]

La lógica recomendada:
- NO uses el rectángulo principal completo como obstáculo.
- Marca manualmente solo las paredes/obstáculos reales.
- En el lado de estacionamientos, simplemente NO marques las salidas como obstáculo.
- Si quieres documentarlas visualmente, usa modo free/opening.

Uso:
    python3 manual_obstacle_rectangle_editor.py \
        --input src/slam/images/map_similarity_crop.png \
        --calibration src/slam/config/map_similarity_calibration.yaml \
        --output-yaml src/slam/config/manual_obstacles.yaml \
        --output-preview src/slam/images/manual_obstacles_preview.png

Controles:
    o  -> modo obstacle
    f  -> modo free/opening/reference
    click + click -> crea rectángulo
    u  -> undo del último rectángulo del modo actual
    r  -> reset completo
    s  -> guardar YAML y preview
    q  -> salir sin guardar

Colores:
    azul  = rectángulo principal de referencia
    verde = obstáculos que sí se mandan a A*
    cyan  = openings/free/reference que NO se mandan a A*
    rojo  = rectángulo en construcción
"""

import argparse
from pathlib import Path

import cv2
import yaml


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def save_yaml(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def get_crop_geometry(calibration):
    scale = float(calibration["basis"]["scale_px_per_m"])

    bounds = calibration["crop"]["world_bounds_m"]
    x_min = float(bounds["x_min"])
    x_max = float(bounds["x_max"])
    y_min = float(bounds["y_min"])
    y_max = float(bounds["y_max"])

    track = calibration["track"]
    width_m = float(track["width_m"])
    height_m = float(track["height_m"])

    return scale, x_min, x_max, y_min, y_max, width_m, height_m


def pixel_to_world(px, py, scale, crop_x_min, crop_y_max):
    x_m = crop_x_min + px / scale
    y_m = crop_y_max - py / scale
    return float(x_m), float(y_m)


def world_to_pixel(x_m, y_m, scale, crop_x_min, crop_y_max):
    px = (x_m - crop_x_min) * scale
    py = (crop_y_max - y_m) * scale
    return int(round(px)), int(round(py))


def normalize_pixel_rect(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return [
        int(min(x1, x2)),
        int(min(y1, y2)),
        int(max(x1, x2)),
        int(max(y1, y2)),
    ]


def pixel_rect_to_world_bbox(rect_px, scale, crop_x_min, crop_y_max, snap_m=0.01):
    x1, y1, x2, y2 = rect_px

    wx1, wy1 = pixel_to_world(x1, y1, scale, crop_x_min, crop_y_max)
    wx2, wy2 = pixel_to_world(x2, y2, scale, crop_x_min, crop_y_max)

    xmin = min(wx1, wx2)
    xmax = max(wx1, wx2)
    ymin = min(wy1, wy2)
    ymax = max(wy1, wy2)

    if snap_m > 0:
        xmin = round(xmin / snap_m) * snap_m
        ymin = round(ymin / snap_m) * snap_m
        xmax = round(xmax / snap_m) * snap_m
        ymax = round(ymax / snap_m) * snap_m

    return [float(xmin), float(ymin), float(xmax), float(ymax)]


def rect_area_m2(bbox_m):
    xmin, ymin, xmax, ymax = bbox_m
    return max(0.0, xmax - xmin) * max(0.0, ymax - ymin)


def draw_text_panel(vis, mode, obstacle_count, free_count):
    lines = [
        f"mode: {mode}",
        f"obstacles: {obstacle_count} | openings/free refs: {free_count}",
        "o: obstacle | f: opening/reference | u: undo | r: reset | s: save | q: quit",
        "click + click = rectangle",
    ]

    y = 24
    for line in lines:
        cv2.putText(
            vis,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 0, 0),
            1,
            cv2.LINE_AA,
        )
        y += 21


def draw_main_track(vis, scale, crop_x_min, crop_y_max, width_m, height_m):
    p_bottom_left = world_to_pixel(0.0, 0.0, scale, crop_x_min, crop_y_max)
    p_top_right = world_to_pixel(width_m, height_m, scale, crop_x_min, crop_y_max)

    x1 = min(p_bottom_left[0], p_top_right[0])
    x2 = max(p_bottom_left[0], p_top_right[0])
    y1 = min(p_bottom_left[1], p_top_right[1])
    y2 = max(p_bottom_left[1], p_top_right[1])

    cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 0, 0), 2)


def draw_rectangles(vis, rects_px, color, alpha=0.18, thickness=2):
    overlay = vis.copy()

    for rect in rects_px:
        x1, y1, x2, y2 = rect
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)

    cv2.addWeighted(overlay, alpha, vis, 1.0 - alpha, 0.0, vis)


class ManualObstacleEditor:
    def __init__(self, image, calibration, snap_m):
        self.image = image
        self.calibration = calibration
        self.snap_m = snap_m

        (
            self.scale,
            self.crop_x_min,
            self.crop_x_max,
            self.crop_y_min,
            self.crop_y_max,
            self.width_m,
            self.height_m,
        ) = get_crop_geometry(calibration)

        self.mode = "obstacle"
        self.obstacles_px = []
        self.free_px = []
        self.current_start = None
        self.mouse_pos = None

        self.window_name = "Manual Obstacle Rectangle Editor"

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_pos = (x, y)

        elif event == cv2.EVENT_LBUTTONDOWN:
            if self.current_start is None:
                self.current_start = (x, y)
                print(f"Inicio rectángulo ({self.mode}): ({x}, {y})")
            else:
                rect = normalize_pixel_rect(self.current_start, (x, y))

                if rect[2] - rect[0] < 2 or rect[3] - rect[1] < 2:
                    print("Rectángulo demasiado pequeño. Ignorado.")
                    self.current_start = None
                    return

                bbox_m = pixel_rect_to_world_bbox(
                    rect,
                    self.scale,
                    self.crop_x_min,
                    self.crop_y_max,
                    self.snap_m,
                )

                if self.mode == "obstacle":
                    self.obstacles_px.append(rect)
                    print(f"Obstáculo agregado px={rect} -> m={bbox_m}")
                else:
                    self.free_px.append(rect)
                    print(f"Opening/reference agregado px={rect} -> m={bbox_m}")

                self.current_start = None

    def render(self):
        if len(self.image.shape) == 2:
            vis = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        else:
            vis = self.image.copy()

        draw_main_track(
            vis,
            self.scale,
            self.crop_x_min,
            self.crop_y_max,
            self.width_m,
            self.height_m,
        )

        draw_rectangles(vis, self.free_px, (255, 255, 0), alpha=0.15, thickness=2)
        draw_rectangles(vis, self.obstacles_px, (0, 255, 0), alpha=0.22, thickness=2)

        if self.current_start is not None and self.mouse_pos is not None:
            temp = normalize_pixel_rect(self.current_start, self.mouse_pos)
            cv2.rectangle(
                vis,
                (temp[0], temp[1]),
                (temp[2], temp[3]),
                (0, 0, 255),
                2,
            )

        draw_text_panel(
            vis,
            self.mode,
            len(self.obstacles_px),
            len(self.free_px),
        )

        return vis

    def undo(self):
        if self.current_start is not None:
            self.current_start = None
            print("Rectángulo en construcción cancelado.")
            return

        if self.mode == "obstacle" and self.obstacles_px:
            removed = self.obstacles_px.pop()
            print(f"Obstáculo eliminado: {removed}")
        elif self.mode == "free" and self.free_px:
            removed = self.free_px.pop()
            print(f"Opening/reference eliminado: {removed}")
        else:
            print("No hay elementos para deshacer en este modo.")

    def reset(self):
        self.current_start = None
        self.obstacles_px = []
        self.free_px = []
        print("Todo reiniciado.")

    def build_output_data(self, input_path):
        obstacles = []
        flat = []

        for idx, rect_px in enumerate(self.obstacles_px):
            bbox_m = pixel_rect_to_world_bbox(
                rect_px,
                self.scale,
                self.crop_x_min,
                self.crop_y_max,
                self.snap_m,
            )

            if rect_area_m2(bbox_m) <= 0.0:
                continue

            flat.extend(bbox_m)
            obstacles.append({
                "id": idx,
                "type": "manual_axis_aligned_rectangle",
                "bbox_m": bbox_m,
                "bbox_format": "[xmin, ymin, xmax, ymax]",
                "source_pixel_rect": rect_px,
            })

        openings = []
        for idx, rect_px in enumerate(self.free_px):
            bbox_m = pixel_rect_to_world_bbox(
                rect_px,
                self.scale,
                self.crop_x_min,
                self.crop_y_max,
                self.snap_m,
            )

            openings.append({
                "id": idx,
                "type": "manual_opening_reference",
                "bbox_m": bbox_m,
                "bbox_format": "[xmin, ymin, xmax, ymax]",
                "source_pixel_rect": rect_px,
            })

        return {
            "description": "Manual obstacle rectangles drawn over calibrated/cropped SLAM map.",
            "source": {
                "input_image": str(input_path),
                "calibration_type": self.calibration.get("calibration_type", "unknown"),
            },
            "map_geometry": {
                "scale_px_per_m": float(self.scale),
                "crop_world_bounds_m": {
                    "x_min": float(self.crop_x_min),
                    "x_max": float(self.crop_x_max),
                    "y_min": float(self.crop_y_min),
                    "y_max": float(self.crop_y_max),
                },
                "main_track_m": {
                    "x_min": 0.0,
                    "x_max": float(self.width_m),
                    "y_min": 0.0,
                    "y_max": float(self.height_m),
                },
            },
            "manual_openings_reference_not_used_by_astar": openings,
            "obstacles": obstacles,
            "obstacles_flat_for_astar": [float(v) for v in flat],
            "ros2_param_example": {
                "hybrid_a_star_bug0_node": {
                    "ros__parameters": {
                        "obstacles": [float(v) for v in flat]
                    }
                }
            },
        }

    def run(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        print("\nManual Obstacle Rectangle Editor")
        print("o -> modo obstacle")
        print("f -> modo opening/reference")
        print("click + click -> crea rectángulo")
        print("u -> undo")
        print("r -> reset")
        print("s -> save")
        print("q -> quit\n")

        save_requested = False

        while True:
            vis = self.render()
            cv2.imshow(self.window_name, vis)
            key = cv2.waitKey(20) & 0xFF

            if key == ord("o"):
                self.mode = "obstacle"
                self.current_start = None
                print("Modo: obstacle")

            elif key == ord("f"):
                self.mode = "free"
                self.current_start = None
                print("Modo: opening/reference")

            elif key == ord("u"):
                self.undo()

            elif key == ord("r"):
                self.reset()

            elif key == ord("s"):
                save_requested = True
                break

            elif key == ord("q"):
                break

        final_preview = self.render()
        cv2.destroyAllWindows()
        return save_requested, final_preview


def main():
    parser = argparse.ArgumentParser(
        description="Editor manual visual para rectángulos de obstáculos."
    )

    parser.add_argument("--input", required=True, help="Imagen crop calibrada.")
    parser.add_argument("--calibration", required=True, help="YAML de calibración similarity.")
    parser.add_argument("--output-yaml", required=True, help="YAML de salida.")
    parser.add_argument("--output-preview", required=True, help="Preview de salida.")
    parser.add_argument("--snap-m", type=float, default=0.01, help="Redondeo en metros.")
    args = parser.parse_args()

    input_path = Path(args.input).expanduser().resolve()
    calibration_path = Path(args.calibration).expanduser().resolve()
    output_yaml_path = Path(args.output_yaml).expanduser().resolve()
    output_preview_path = Path(args.output_preview).expanduser().resolve()

    image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"No se pudo leer la imagen: {input_path}")

    calibration = load_yaml(calibration_path)

    editor = ManualObstacleEditor(
        image=image,
        calibration=calibration,
        snap_m=args.snap_m,
    )

    save_requested, preview = editor.run()

    if not save_requested:
        print("Saliendo sin guardar.")
        return

    output_data = editor.build_output_data(input_path)
    save_yaml(output_yaml_path, output_data)

    output_preview_path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(output_preview_path), preview)
    if not ok:
        raise RuntimeError(f"No se pudo guardar preview: {output_preview_path}")

    print("\nGuardado exitosamente.")
    print(f"YAML: {output_yaml_path}")
    print(f"Preview: {output_preview_path}")
    print(f"Obstáculos: {len(output_data['obstacles'])}")
    print(f"Openings/reference: {len(output_data['manual_openings_reference_not_used_by_astar'])}")
    print("\nA* flat obstacles:")
    print(output_data["obstacles_flat_for_astar"])


if __name__ == "__main__":
    main()
