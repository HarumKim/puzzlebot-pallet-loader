#!/usr/bin/env python3
"""
map_orientation_editor_v2.py

Editor interactivo para corregir orientación de mapas SLAM sin deformar la imagen.

Mejora principal:
- No aplica rotaciones finas acumulativas sobre la imagen ya rotada.
- Mantiene una imagen base y recalcula la vista desde cero cada vez.
- Evita el efecto de deformación/espiral por interpolaciones repetidas.

Uso:
    python3 map_orientation_editor_v2.py --input src/slam/images/map.png --output src/slam/images/map_oriented.png

Controles:
    r  -> rotar base 90° derecha
    l  -> rotar base 90° izquierda
    u  -> rotar base 180°
    h  -> flip horizontal sobre la base
    v  -> flip vertical sobre la base
    +  -> aumentar rotación fina +1°
    -  -> disminuir rotación fina -1°
    0  -> resetear rotación fina a 0°
    s  -> guardar imagen actual
    q  -> salir sin guardar
"""

import argparse
from pathlib import Path

import cv2


def rotate_keep_canvas(image, angle_deg, fill_value=128):
    """Rota alrededor del centro manteniendo el mismo tamaño del canvas."""
    h, w = image.shape[:2]
    center = (w / 2.0, h / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle_deg, 1.0)

    return cv2.warpAffine(
        image,
        matrix,
        (w, h),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=fill_value
    )


def add_overlay(image, fine_angle):
    if len(image.shape) == 2:
        preview = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        preview = image.copy()

    cv2.putText(
        preview,
        f"fine angle: {fine_angle:.1f} deg",
        (15, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (0, 0, 255),
        1,
        cv2.LINE_AA
    )

    cv2.putText(
        preview,
        "r/l/u rotate | h/v flip | +/- fine | 0 reset | s save | q quit",
        (15, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (0, 0, 255),
        1,
        cv2.LINE_AA
    )

    return preview


def print_help():
    print("\nControles:")
    print("  r  -> rotar base 90° derecha")
    print("  l  -> rotar base 90° izquierda")
    print("  u  -> rotar base 180°")
    print("  h  -> flip horizontal sobre la base")
    print("  v  -> flip vertical sobre la base")
    print("  +  -> rotación fina +1°")
    print("  -  -> rotación fina -1°")
    print("  0  -> resetear rotación fina")
    print("  s  -> guardar")
    print("  q  -> salir sin guardar\n")


def main():
    parser = argparse.ArgumentParser(
        description="Editor de orientación para mapas SLAM sin deformación acumulada."
    )
    parser.add_argument("--input", required=True, help="Ruta de imagen de entrada")
    parser.add_argument("--output", default=None, help="Ruta de imagen de salida")
    args = parser.parse_args()

    input_path = Path(args.input).expanduser().resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"No existe la imagen: {input_path}")

    if args.output is None:
        output_path = input_path.with_name(input_path.stem + "_oriented" + input_path.suffix)
    else:
        output_path = Path(args.output).expanduser().resolve()

    original = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)
    if original is None:
        raise RuntimeError(f"No se pudo leer la imagen: {input_path}")

    base_image = original.copy()
    fine_angle = 0.0

    print(f"Imagen cargada: {input_path}")
    print(f"Salida: {output_path}")
    print_help()

    window_name = "Map Orientation Editor v2"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    while True:
        current = rotate_keep_canvas(base_image, fine_angle) if abs(fine_angle) > 1e-6 else base_image.copy()
        preview = add_overlay(current, fine_angle)

        cv2.imshow(window_name, preview)
        key = cv2.waitKey(0) & 0xFF

        if key == ord("r"):
            base_image = cv2.rotate(base_image, cv2.ROTATE_90_CLOCKWISE)
            fine_angle = 0.0
            print("Base rotada 90° derecha. Rotación fina reseteada.")

        elif key == ord("l"):
            base_image = cv2.rotate(base_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            fine_angle = 0.0
            print("Base rotada 90° izquierda. Rotación fina reseteada.")

        elif key == ord("u"):
            base_image = cv2.rotate(base_image, cv2.ROTATE_180)
            fine_angle = 0.0
            print("Base rotada 180°. Rotación fina reseteada.")

        elif key == ord("h"):
            base_image = cv2.flip(base_image, 1)
            print("Flip horizontal aplicado sobre base.")

        elif key == ord("v"):
            base_image = cv2.flip(base_image, 0)
            print("Flip vertical aplicado sobre base.")

        elif key == ord("+") or key == ord("="):
            fine_angle += 1.0
            print(f"Rotación fina: {fine_angle:.1f}°")

        elif key == ord("-") or key == ord("_"):
            fine_angle -= 1.0
            print(f"Rotación fina: {fine_angle:.1f}°")

        elif key == ord("0"):
            fine_angle = 0.0
            print("Rotación fina reseteada a 0°.")

        elif key == ord("s"):
            final_image = rotate_keep_canvas(base_image, fine_angle) if abs(fine_angle) > 1e-6 else base_image.copy()
            output_path.parent.mkdir(parents=True, exist_ok=True)

            ok = cv2.imwrite(str(output_path), final_image)
            if not ok:
                raise RuntimeError(f"No se pudo guardar la imagen en: {output_path}")

            print(f"Imagen guardada en: {output_path}")
            break

        elif key == ord("q"):
            print("Saliendo sin guardar.")
            break

        else:
            print_help()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()