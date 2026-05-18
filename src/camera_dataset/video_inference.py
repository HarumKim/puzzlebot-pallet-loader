import cv2
import glob
import os
from ultralytics import YOLO

DATASET_DIR = os.path.join(os.path.dirname(__file__), "dataset", "dataset")
MODEL_PATH = os.path.join(os.path.dirname(__file__), "best.pt")
FPS = 10
CONF_THRESHOLD = 0.25

model = YOLO(MODEL_PATH)
print(f"Model loaded: {MODEL_PATH}")
print(f"Classes: {model.names}")

image_paths = sorted(glob.glob(os.path.join(DATASET_DIR, "image_*.jpg")))
if not image_paths:
    raise FileNotFoundError(f"No images found in {DATASET_DIR}")
print(f"Found {len(image_paths)} images")

first = cv2.imread(image_paths[0])
h, w = first.shape[:2]

cv2.namedWindow("YOLO11 Inference", cv2.WINDOW_NORMAL)
cv2.resizeWindow("YOLO11 Inference", min(w, 1280), min(h, 720))

delay = int(1000 / FPS)

for idx, path in enumerate(image_paths):
    frame = cv2.imread(path)
    if frame is None:
        continue

    results = model(frame, conf=CONF_THRESHOLD, verbose=False)
    annotated = results[0].plot()

    label = f"Frame {idx+1}/{len(image_paths)}  |  {os.path.basename(path)}  |  Press Q to quit"
    cv2.putText(annotated, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("YOLO11 Inference", annotated)

    key = cv2.waitKey(delay) & 0xFF
    if key == ord("q") or key == 27:
        print("Interrupted by user.")
        break

cv2.destroyAllWindows()
print("Done.")
