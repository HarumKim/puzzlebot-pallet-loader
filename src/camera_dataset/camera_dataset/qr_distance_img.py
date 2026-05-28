import cv2
import numpy as np

# ── CONFIGURACIÓN ──────────────────────────────────────────────────────────
import os
_PKG = os.path.join(os.path.dirname(__file__), '..')

CALIB_FILE  = os.path.join(_PKG, 'camera_calibration.npz')
IMAGE_PATH  = os.path.join(_PKG, 'qr_test_image.jpg')
QR_WIDTH    = 0.12   # 12 cm en metros
QR_HEIGHT   = 0.11   # 11 cm en metros
# ──────────────────────────────────────────────────────────────────────────

calib       = np.load(CALIB_FILE)
CAMERA_MAT  = calib['camera_matrix']
DIST_COEFFS = calib['dist_coeffs']

QR_3D_POINTS = np.array([
    [0,        0,         0],
    [QR_WIDTH, 0,         0],
    [QR_WIDTH, QR_HEIGHT, 0],
    [0,        QR_HEIGHT, 0],
], dtype=np.float32)

frame = cv2.imread(IMAGE_PATH)
if frame is None:
    print(f"ERROR: no se pudo cargar {IMAGE_PATH}")
    exit()

qr = cv2.QRCodeDetector()
data, points, _ = qr.detectAndDecode(frame)

if points is None or len(points) == 0:
    print("QR no detectado en la imagen")
    cv2.imshow('resultado', frame)
    cv2.waitKey(0)
    exit()

print(f"QR detectado: '{data}'")

image_points = points[0].astype(np.float32)

success, rvec, tvec = cv2.solvePnP(
    QR_3D_POINTS,
    image_points,
    CAMERA_MAT,
    DIST_COEFFS,
    flags=cv2.SOLVEPNP_IPPE_SQUARE
)

if not success:
    print("solvePnP falló")
    exit()

x = float(tvec[0])
y = float(tvec[1])
z = float(tvec[2])

print(f"\nResultados:")
print(f"  Distancia al pallet : {z:.3f} m")
print(f"  Error lateral       : {x:.3f} m  ({'derecha' if x > 0 else 'izquierda'})")
print(f"  Altura del QR       : {y:.3f} m")
print(f"\n  → Subir forklift a  : {y:.3f} m")
print(f"  → Avanzar           : {z:.3f} m")

pts = image_points.astype(int)
cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
cv2.putText(frame,
    f"dist={z:.2f}m",
    (pts[0][0], pts[0][1] - 30),
    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
cv2.putText(frame,
    f"lateral={x:.2f}m  altura={y:.2f}m",
    (pts[0][0], pts[0][1] - 10),
    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

cv2.imshow('QR Detection', frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
