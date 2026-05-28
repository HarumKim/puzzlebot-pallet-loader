#!/usr/bin/env python3
"""
map_similarity_calibration_click.py

Calibra un mapa SLAM usando rototraslación con escala uniforme.

Este enfoque está pensado para mapas generados por SLAM que ya tienen vista superior.
No usa homografía ni fuerza 4 puntos manuales, para evitar deformaciones.

Clicks necesarios:
    1) origin_0_0:
       esquina física elegida como (0.0, 0.0)

    2) x_axis_end:
       punto sobre el eje X positivo, correspondiente a (track_width_m, 0.0)
       Por default: (4.84, 0.0)

    3) y_side_hint:
       cualquier punto hacia el lado donde debe crecer Y positivo.
       No tiene que ser esquina exacta; solo indica hacia qué lado construir el rectángulo.

El script:
- Calcula escala uniforme px/m.
- Calcula ejes ortogonales X/Y.
- Construye un rectángulo ideal de 4.84 x 3.67 m.
- Recorta la imagen tomando el rectángulo principal + margen en metros.
- Guarda YAML de calibración.
- Guarda imagen preview anotada.
- Guarda imagen recortada.

Uso:
    python3 map_similarity_calibration_click.py \
        --input src/slam/images/map_oriented.png \
        --output-yaml src/slam/config/map_similarity_calibration.yaml \
        --output-image src/slam/images/map_similarity_preview.png \
        --output-crop src/slam/images/map_similarity_crop.png \
        --margin-m 1.0

Controles:
    u -> deshacer último punto
    r -> reiniciar puntos
    s -> guardar calibración y recorte
    q -> salir sin guardar
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


TRACK_WIDTH_M = 4.84
TRACK_HEIGHT_M = 3.67
DEFAULT_MARGIN_M = 1.0

POINT_NAMES = [
    "origin_0_0",
    "x_axis_end",
    "y_side_hint",
]


def normalize(v):
    norm = np.linalg.norm(v)
    if norm < 1e-9:
        raise ValueError("Vector demasiado pequeño para normalizar.")
    return v / norm


def compute_similarity_calibration(points_px, width_m, height_m, margin_m):
    """
    Calcula calibración de similitud imagen <-> mundo.

    p_img = origin_px + scale_px_per_m * (x_m * vx + y_m * vy)

    Inversa:
    delta = p_img - origin_px
    x_m = dot(delta, vx) / scale_px_per_m
    y_m = dot(delta, vy) / scale_px_per_m
    """
    if len(points_px) != 3:
        raise ValueError("Se necesitan exactamente 3 puntos.")

    origin = np.array(points_px[0], dtype=np.float64)
    x_axis_end = np.array(points_px[1], dtype=np.float64)
    y_hint = np.array(points_px[2], dtype=np.float64)

    x_vec = x_axis_end - origin
    x_len_px = np.linalg.norm(x_vec)

    if x_len_px < 1e-6:
        raise ValueError("El punto origin_0_0 y x_axis_end están demasiado cerca.")

    vx = normalize(x_vec)

    # Dos posibles perpendiculares para Y.
    vy_candidate_1 = np.array([-vx[1], vx[0]], dtype=np.float64)
    vy_candidate_2 = np.array([vx[1], -vx[0]], dtype=np.float64)

    hint_vec = y_hint - origin

    # Elegimos el candidato que apunta hacia el lado donde hiciste el tercer click.
    if np.dot(hint_vec, vy_candidate_1) >= 0:
        vy = vy_candidate_1
    else:
        vy = vy_candidate_2

    scale_px_per_m = x_len_px / width_m
    meters_per_pixel = 1.0 / scale_px_per_m

    # Rectángulo principal ideal en pixeles.
    p0 = origin
    p1 = origin + scale_px_per_m * width_m * vx
    p2 = origin + scale_px_per_m * (width_m * vx + height_m * vy)
    p3 = origin + scale_px_per_m * height_m * vy

    rectangle_px = np.vstack([p0, p1, p2, p3])

    # Rectángulo extendido con margen en metros.
    # Coordenadas mundo con margen:
    # x in [-margin, width + margin]
    # y in [-margin, height + margin]
    crop_world = np.array([
        [-margin_m, -margin_m],
        [width_m + margin_m, -margin_m],
        [width_m + margin_m, height_m + margin_m],
        [-margin_m, height_m + margin_m],
    ], dtype=np.float64)

    crop_px = np.array([
        world_to_image_xy(x, y, origin, vx, vy, scale_px_per_m)
        for x, y in crop_world
    ], dtype=np.float64)

    return {
        "origin_px": origin,
        "x_axis_end_px": x_axis_end,
        "y_side_hint_px": y_hint,
        "vx": vx,
        "vy": vy,
        "scale_px_per_m": float(scale_px_per_m),
        "meters_per_pixel": float(meters_per_pixel),
        "rectangle_px": rectangle_px,
        "crop_world_m": crop_world,
        "crop_px": crop_px,
    }


def image_to_world_xy(px, py, origin_px, vx, vy, scale_px_per_m):
    p = np.array([px, py], dtype=np.float64)
    delta = p - origin_px

    x_m = np.dot(delta, vx) / scale_px_per_m
    y_m = np.dot(delta, vy) / scale_px_per_m

    return float(x_m), float(y_m)


def world_to_image_xy(x_m, y_m, origin_px, vx, vy, scale_px_per_m):
    p = origin_px + scale_px_per_m * (x_m * vx + y_m * vy)
    return np.array([float(p[0]), float(p[1])], dtype=np.float64)


def crop_rotated_region(image, calibration, output_width_m, output_height_m):
    """
    Genera una imagen recortada y alineada con el sistema mundo.

    El recorte incluye:
        x in [-margin, width + margin]
        y in [-margin, height + margin]

    El resultado queda con X hacia la derecha y Y hacia arriba en mundo.
    En imagen, Y crece hacia abajo, por eso se construye de top-left a bottom-right.
    """
    scale = calibration["scale_px_per_m"]
    origin = calibration["origin_px"]
    vx = calibration["vx"]
    vy = calibration["vy"]

    out_w_px = int(round(output_width_m * scale))
    out_h_px = int(round(output_height_m * scale))

    # Queremos que la imagen de salida muestre:
    # top-left     = mundo (-margin, height+margin)
    # top-right    = mundo (width+margin, height+margin)
    # bottom-right = mundo (width+margin, -margin)
    # bottom-left  = mundo (-margin, -margin)
    margin_world = calibration["crop_world_m"]

    x_min = float(np.min(margin_world[:, 0]))
    x_max = float(np.max(margin_world[:, 0]))
    y_min = float(np.min(margin_world[:, 1]))
    y_max = float(np.max(margin_world[:, 1]))

    src = np.array([
        world_to_image_xy(x_min, y_max, origin, vx, vy, scale),  # top-left in output
        world_to_image_xy(x_max, y_max, origin, vx, vy, scale),  # top-right
        world_to_image_xy(x_max, y_min, origin, vx, vy, scale),  # bottom-right
        world_to_image_xy(x_min, y_min, origin, vx, vy, scale),  # bottom-left
    ], dtype=np.float32)

    dst = np.array([
        [0, 0],
        [out_w_px - 1, 0],
        [out_w_px - 1, out_h_px - 1],
        [0, out_h_px - 1],
    ], dtype=np.float32)

    M = cv2.getPerspectiveTransform(src, dst)

    cropped = cv2.warpPerspective(
        image,
        M,
        (out_w_px, out_h_px),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=128,
    )

    return cropped, M


def draw_poly(vis, pts, color, thickness=2):
    pts_i = np.round(pts).astype(np.int32)
    for i in range(len(pts_i)):
        p1 = tuple(pts_i[i])
        p2 = tuple(pts_i[(i + 1) % len(pts_i)])
        cv2.line(vis, p1, p2, color, thickness)


def draw_preview(image, points, calibration=None):
    if len(image.shape) == 2:
        vis = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        vis = image.copy()

    for i, (x, y) in enumerate(points):
        cv2.circle(vis, (int(x), int(y)), 5, (0, 0, 255), -1)
        cv2.putText(
            vis,
            f"{i + 1}: {POINT_NAMES[i]}",
            (int(x) + 8, int(y) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    if calibration is not None:
        draw_poly(vis, calibration["rectangle_px"], (0, 255, 255), 2)
        draw_poly(vis, calibration["crop_px"], (255, 0, 255), 2)

        origin = calibration["origin_px"]
        vx = calibration["vx"]
        vy = calibration["vy"]
        scale = calibration["scale_px_per_m"]

        x_arrow = origin + 0.75 * scale * vx
        y_arrow = origin + 0.75 * scale * vy

        cv2.arrowedLine(
            vis,
            tuple(np.round(origin).astype(int)),
            tuple(np.round(x_arrow).astype(int)),
            (0, 255, 0),
            2,
            tipLength=0.2,
        )
        cv2.putText(
            vis,
            "X+",
            tuple(np.round(x_arrow).astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.arrowedLine(
            vis,
            tuple(np.round(origin).astype(int)),
            tuple(np.round(y_arrow).astype(int)),
            (255, 0, 0),
            2,
            tipLength=0.2,
        )
        cv2.putText(
            vis,
            "Y+",
            tuple(np.round(y_arrow).astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )

    help_text = [
        "Click 3 points:",
        "1 origin_0_0 = (0,0)",
        "2 x_axis_end = (4.84,0)",
        "3 y_side_hint = side where Y grows",
        "yellow: main 4.84x3.67m | magenta: crop + margin",
        "u: undo | r: reset | s: save | q: quit",
    ]

    y0 = 25
    for line in help_text:
        cv2.putText(
            vis,
            line,
            (15, y0),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 0, 0),
            1,
            cv2.LINE_AA,
        )
        y0 += 22

    return vis


def build_yaml(calibration, image_shape, width_m, height_m, margin_m, crop_matrix):
    h, w = image_shape[:2]

    origin = calibration["origin_px"]
    vx = calibration["vx"]
    vy = calibration["vy"]

    data = {
        "description": "SLAM map similarity calibration: rotation + translation + uniform scale.",
        "calibration_type": "similarity_transform_uniform_scale",
        "image": {
            "width_px": int(w),
            "height_px": int(h),
        },
        "track": {
            "width_m": float(width_m),
            "height_m": float(height_m),
            "crop_margin_m": float(margin_m),
            "origin": "origin_0_0",
            "x_axis": "origin_0_0_to_x_axis_end",
            "y_axis": "orthogonal_axis_selected_by_y_side_hint",
        },
        "clicked_points_px": {
            "origin_0_0": calibration["origin_px"].astype(float).tolist(),
            "x_axis_end": calibration["x_axis_end_px"].astype(float).tolist(),
            "y_side_hint": calibration["y_side_hint_px"].astype(float).tolist(),
        },
        "basis": {
            "origin_px": origin.astype(float).tolist(),
            "vx_image_unit": vx.astype(float).tolist(),
            "vy_image_unit": vy.astype(float).tolist(),
            "scale_px_per_m": float(calibration["scale_px_per_m"]),
            "meters_per_pixel": float(calibration["meters_per_pixel"]),
        },
        "main_rectangle_px": {
            "origin_0_0": calibration["rectangle_px"][0].astype(float).tolist(),
            "x_axis_end": calibration["rectangle_px"][1].astype(float).tolist(),
            "opposite_corner": calibration["rectangle_px"][2].astype(float).tolist(),
            "y_axis_end": calibration["rectangle_px"][3].astype(float).tolist(),
        },
        "crop": {
            "world_bounds_m": {
                "x_min": float(np.min(calibration["crop_world_m"][:, 0])),
                "x_max": float(np.max(calibration["crop_world_m"][:, 0])),
                "y_min": float(np.min(calibration["crop_world_m"][:, 1])),
                "y_max": float(np.max(calibration["crop_world_m"][:, 1])),
            },
            "crop_corners_px": {
                "bottom_left": calibration["crop_px"][0].astype(float).tolist(),
                "bottom_right": calibration["crop_px"][1].astype(float).tolist(),
                "top_right": calibration["crop_px"][2].astype(float).tolist(),
                "top_left": calibration["crop_px"][3].astype(float).tolist(),
            },
            "warp_matrix_original_image_to_crop_image": crop_matrix.astype(float).tolist(),
        },
        "formulas": {
            "image_to_world": [
                "delta = p_img - origin_px",
                "x_m = dot(delta, vx_image_unit) / scale_px_per_m",
                "y_m = dot(delta, vy_image_unit) / scale_px_per_m",
            ],
            "world_to_image": [
                "p_img = origin_px + scale_px_per_m * (x_m * vx_image_unit + y_m * vy_image_unit)"
            ],
        },
    }

    return data


class SimilarityCalibrator:
    def __init__(self, image, width_m, height_m, margin_m):
        self.image = image
        self.width_m = width_m
        self.height_m = height_m
        self.margin_m = margin_m
        self.points = []
        self.window_name = "Similarity Map Calibration"

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.points) < 3:
                self.points.append((float(x), float(y)))
                print(f"Punto {len(self.points)} agregado: {POINT_NAMES[len(self.points)-1]} = ({x}, {y})")
            else:
                print("Ya hay 3 puntos. Presiona 'r' para reiniciar o 's' para guardar.")

    def run(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        print("\nSelecciona 3 puntos:")
        print("  1) origin_0_0")
        print("  2) x_axis_end, punto que representa 4.84 m sobre X+")
        print("  3) y_side_hint, cualquier punto hacia donde debe crecer Y+")
        print("\nTeclas:")
        print("  u -> deshacer último punto")
        print("  r -> reiniciar")
        print("  s -> guardar")
        print("  q -> salir\n")

        while True:
            calibration = None
            if len(self.points) == 3:
                try:
                    calibration = compute_similarity_calibration(
                        self.points,
                        self.width_m,
                        self.height_m,
                        self.margin_m,
                    )
                except Exception as e:
                    print(f"No se pudo calcular preview: {e}")

            vis = draw_preview(self.image, self.points, calibration)
            cv2.imshow(self.window_name, vis)
            key = cv2.waitKey(20) & 0xFF

            if key == ord("u"):
                if self.points:
                    removed = self.points.pop()
                    print(f"Punto eliminado: {removed}")

            elif key == ord("r"):
                self.points = []
                print("Puntos reiniciados.")

            elif key == ord("s"):
                if len(self.points) != 3:
                    print("Necesitas seleccionar exactamente 3 puntos antes de guardar.")
                    continue
                break

            elif key == ord("q"):
                self.points = []
                break

        cv2.destroyAllWindows()
        return self.points


def main():
    parser = argparse.ArgumentParser(
        description="Calibración por rototraslación + escala uniforme para mapa SLAM."
    )
    parser.add_argument("--input", required=True, help="Imagen de mapa orientada.")
    parser.add_argument("--output-yaml", default="map_similarity_calibration.yaml")
    parser.add_argument("--output-image", default=None, help="Preview anotado.")
    parser.add_argument("--output-crop", default=None, help="Imagen recortada y alineada.")
    parser.add_argument("--width-m", type=float, default=TRACK_WIDTH_M)
    parser.add_argument("--height-m", type=float, default=TRACK_HEIGHT_M)
    parser.add_argument("--margin-m", type=float, default=DEFAULT_MARGIN_M)
    args = parser.parse_args()

    input_path = Path(args.input).expanduser().resolve()
    output_yaml = Path(args.output_yaml).expanduser().resolve()

    if args.output_image is None:
        output_image = input_path.with_name(input_path.stem + "_similarity_preview.png")
    else:
        output_image = Path(args.output_image).expanduser().resolve()

    if args.output_crop is None:
        output_crop = input_path.with_name(input_path.stem + "_similarity_crop.png")
    else:
        output_crop = Path(args.output_crop).expanduser().resolve()

    image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"No se pudo leer la imagen: {input_path}")

    calibrator = SimilarityCalibrator(
        image=image,
        width_m=args.width_m,
        height_m=args.height_m,
        margin_m=args.margin_m,
    )

    points = calibrator.run()

    if len(points) != 3:
        print("No se guardó calibración porque no se seleccionaron 3 puntos.")
        return

    calibration = compute_similarity_calibration(
        points,
        args.width_m,
        args.height_m,
        args.margin_m,
    )

    crop_width_m = args.width_m + 2.0 * args.margin_m
    crop_height_m = args.height_m + 2.0 * args.margin_m

    cropped, crop_matrix = crop_rotated_region(
        image,
        calibration,
        crop_width_m,
        crop_height_m,
    )

    yaml_data = build_yaml(
        calibration=calibration,
        image_shape=image.shape,
        width_m=args.width_m,
        height_m=args.height_m,
        margin_m=args.margin_m,
        crop_matrix=crop_matrix,
    )

    preview = draw_preview(image, points, calibration)

    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    output_image.parent.mkdir(parents=True, exist_ok=True)
    output_crop.parent.mkdir(parents=True, exist_ok=True)

    with open(output_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(yaml_data, f, sort_keys=False, allow_unicode=True)

    cv2.imwrite(str(output_image), preview)
    cv2.imwrite(str(output_crop), cropped)

    print("\nCalibración guardada correctamente.")
    print(f"YAML: {output_yaml}")
    print(f"Preview: {output_image}")
    print(f"Crop: {output_crop}")
    print("\nParámetros clave:")
    print(f"  scale_px_per_m: {calibration['scale_px_per_m']:.6f}")
    print(f"  meters_per_pixel: {calibration['meters_per_pixel']:.6f}")
    print(f"  crop size meters: {crop_width_m:.2f} x {crop_height_m:.2f}")
    print(f"  crop size pixels: {cropped.shape[1]} x {cropped.shape[0]}")

    print("\nPrueba rápida:")
    for px, py in points:
        x_m, y_m = image_to_world_xy(
            px,
            py,
            calibration["origin_px"],
            calibration["vx"],
            calibration["vy"],
            calibration["scale_px_per_m"],
        )
        print(f"  pixel=({px:.1f}, {py:.1f}) -> world=({x_m:.3f}, {y_m:.3f}) m")


if __name__ == "__main__":
    main()
