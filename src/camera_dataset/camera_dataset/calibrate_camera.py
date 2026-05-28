import cv2
import numpy as np
import glob
import os

# ── CONFIGURACIÓN ──────────────────────────────────────────────────────────
BOARD_SIZE   = (7, 5)        # esquinas internas: 7 columnas x 5 filas
SQUARE_SIZE  = 0.03          # tamaño de cada cuadro en metros (3 cm)
IMAGES_PATH  = '/home/kim/datasets/checkboard/*.jpg'
OUTPUT_FILE  = '/home/kim/datasets/checkboard/camera_calibration.npz'
# ──────────────────────────────────────────────────────────────────────────

# Puntos 3D del tablero en el mundo real
objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []   # puntos 3D reales
imgpoints = []   # puntos 2D en imagen

images = sorted(glob.glob(IMAGES_PATH))
print(f"Fotos encontradas: {len(images)}")

valid = 0
for fname in images:
    img  = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

    if ret:
        # refinar esquinas para mayor precisión
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners2)
        valid += 1
        print(f"✓  {os.path.basename(fname)}")
    else:
        print(f"✗  {os.path.basename(fname)} — tablero no detectado")

print(f"\nFotos válidas: {valid}/{len(images)}")

if valid < 10:
    print("ERROR: necesitas al menos 10 fotos válidas para calibrar")
    exit()

# ── Calibración ────────────────────────────────────────────────────────────
print("\nCalibrando...")
h, w = cv2.imread(images[0]).shape[:2]
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, (w, h), None, None
)

print(f"\nError de reproyección: {ret:.4f} px")
print("(menor a 1.0 es bueno, menor a 0.5 es excelente)")

print("\nMatriz intrínseca (camera_matrix):")
print(camera_matrix)

print("\nCoeficientes de distorsión:")
print(dist_coeffs)

# ── Guardar resultados ─────────────────────────────────────────────────────
np.savez(OUTPUT_FILE,
         camera_matrix=camera_matrix,
         dist_coeffs=dist_coeffs,
         reprojection_error=ret)

print(f"\nCalibración guardada en: {OUTPUT_FILE}")