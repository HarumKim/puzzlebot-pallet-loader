"""
Entrenamiento.py — Pipeline completo de preparación
Ejecutar UNA SOLA VEZ para generar todos los archivos .pkl necesarios.

Etapas:
    1. Extracción de MFCC + construcción del Codebook 
    2. Cuantización vectorial de todas las grabaciones
    3. Inicialización de los 10 modelos HMM (topología Bakis left-to-right)
    4. Entrenamiento por segmentación lineal y conteos directos (hold-out 80/20)

Archivos generados:
    codebook_256.pkl
    secuencias_cuantizadas.pkl
    modelos_hmm_iniciales.pkl
    modelos_hmm_entrenados.pkl
"""

import os
import pickle
import numpy as np
import librosa
from sklearn.cluster import KMeans

# CONFIGURACIÓN GLOBAL
FS          = 16000
N_MFCC      = 13
HOP_LENGTH  = 512
N_CLUSTERS  = 256
N_ESTADOS   = 5
EPSILON     = 1e-6
BASE_DIR    = "DATASET"
PALABRAS    = ["start", "stop", "lift", "drop", "left",
               "right", "forward", "back", "fast", "slow"]

FILE_CODEBOOK       = "codebook_256.pkl"
FILE_SEQUENCES      = "secuencias_cuantizadas.pkl"
FILE_MODELS_INIT    = "modelos_hmm_iniciales.pkl"
FILE_MODELS_TRAINED = "modelos_hmm_entrenados.pkl"


# ETAPA 1 — EXTRACCIÓN DE MFCC Y CODEBOOK
print("=" * 60)
print("ETAPA 1: Extracción de MFCC y construcción del Codebook")
print("=" * 60)

todos_los_frames = []
dataset_estructurado = {}

for word in os.listdir(BASE_DIR):
    word_dir = os.path.join(BASE_DIR, word)
    if not os.path.isdir(word_dir):
        continue

    dataset_estructurado[word] = []
    print(f"-> Procesando palabra: '{word.upper()}'")

    for filename in os.listdir(word_dir):
        if not filename.endswith(".wav"):
            continue

        path = os.path.join(word_dir, filename)
        y, sr = librosa.load(path, sr=FS)
        y_trimmed, _ = librosa.effects.trim(y, top_db=20)
        mfcc = librosa.feature.mfcc(
            y=y_trimmed, sr=sr, n_mfcc=N_MFCC, hop_length=HOP_LENGTH
        )
        mfcc_frames = mfcc.T  # shape: (n_frames, N_MFCC)

        todos_los_frames.append(mfcc_frames)
        dataset_estructurado[word].append({
            "filename": filename,
            "mfcc": mfcc_frames
        })

X_train = np.vstack(todos_los_frames)
print(f"\nTotal de vectores (frames) para entrenar el Codebook: {X_train.shape[0]}")

print(f"\nEntrenando Codebook de {N_CLUSTERS} vectores ...")
kmeans = KMeans(n_clusters=N_CLUSTERS, random_state=42, n_init=10)
kmeans.fit(X_train)

with open(FILE_CODEBOOK, 'wb') as f:
    pickle.dump(kmeans, f)
print(f"[✓] Codebook guardado en '{FILE_CODEBOOK}'")

# ETAPA 2 — CUANTIZACIÓN VECTORIAL
print("\n" + "=" * 60)
print("ETAPA 2: Cuantización vectorial (MFCC → índices 0-255)")
print("=" * 60)

secuencias_finales = {}

for word, grabaciones in dataset_estructurado.items():
    secuencias_finales[word] = []
    print(f"-> Cuantizando palabra: '{word.upper()}'")

    for grabacion in grabaciones:
        secuencia_enteros = kmeans.predict(grabacion["mfcc"]).tolist()
        secuencias_finales[word].append({
            "archivo": grabacion["filename"],
            "observacion_O": secuencia_enteros
        })

with open(FILE_SEQUENCES, 'wb') as f:
    pickle.dump(secuencias_finales, f)
print(f"[✓] Secuencias guardadas en '{FILE_SEQUENCES}'")

# Ejemplo de salida
palabra_ejemplo = list(secuencias_finales.keys())[0]
ejemplo = secuencias_finales[palabra_ejemplo][0]
print(f"\nEjemplo — '{palabra_ejemplo.upper()}' ({ejemplo['archivo']}):")
print(f"O = {ejemplo['observacion_O'][:15]}...  (longitud: {len(ejemplo['observacion_O'])} frames)")

# ETAPA 3 — INICIALIZACIÓN DE MODELOS HMM
print("\n" + "=" * 60)
print("ETAPA 3: Inicialización de topología Bakis (Left-to-Right)")
print("=" * 60)

modelos_hmm = {}

for palabra in PALABRAS:
    # Matriz A: 50% quedarse, 50% avanzar; último estado absorbente
    A = np.zeros((N_ESTADOS, N_ESTADOS))
    for i in range(N_ESTADOS - 1):
        A[i, i]   = 0.5
        A[i, i+1] = 0.5
    A[N_ESTADOS - 1, N_ESTADOS - 1] = 1.0

    # Matriz B: distribución uniforme sobre 256 símbolos
    B = np.full((N_ESTADOS, N_CLUSTERS), 1.0 / N_CLUSTERS)

    # Vector pi: siempre se empieza en el estado 0
    pi = np.zeros(N_ESTADOS)
    pi[0] = 1.0

    modelos_hmm[palabra] = {"A": A, "B": B, "pi": pi, "n_estados": N_ESTADOS}
    print(f"-> Modelo inicializado: '{palabra.upper()}'")

with open(FILE_MODELS_INIT, 'wb') as f:
    pickle.dump(modelos_hmm, f)
print(f"[✓] Modelos iniciales guardados en '{FILE_MODELS_INIT}'")

# ETAPA 4 — ENTRENAMIENTO POR SEGMENTACIÓN LINEAL
print("\n" + "=" * 60)
print("ETAPA 4: Entrenamiento HMM — Segmentación Lineal y Suavizado")
print("=" * 60)

for palabra, grabaciones in secuencias_finales.items():
    print(f"-> Entrenando modelo para la palabra: '{palabra.upper()}'")

    n_estados = modelos_hmm[palabra]["n_estados"]

    # Acumuladores de conteos
    conteos_B          = np.zeros((n_estados, N_CLUSTERS))
    conteos_A_quedarse = np.zeros(n_estados - 1)
    conteos_A_avanzar  = np.zeros(n_estados - 1)

        # Hold-out 80/20: solo las primeras 48 grabaciones para entrenar
    split = int(len(grabaciones) * 0.8)
    grabaciones_train = grabaciones[:split]   # ← 80% para entrenar

    for grabacion in grabaciones_train:
        O = grabacion["observacion_O"]
        T = len(O)

        # Segmentación lineal en N_ESTADOS partes iguales
        limites = np.linspace(0, T, n_estados + 1).astype(int)

        for j in range(n_estados):
            segmento_O = O[limites[j]:limites[j + 1]]

            # Conteos para B
            for simbolo in segmento_O:
                conteos_B[j, simbolo] += 1

            # Conteos para A (solo estados que pueden avanzar)
            if j < n_estados - 1:
                duracion = len(segmento_O)
                if duracion > 0:
                    conteos_A_quedarse[j] += max(0, duracion - 1)
                    conteos_A_avanzar[j]  += 1

    # Actualizar B con suavizado epsilon y normalización exacta
    B_suavizada  = conteos_B + EPSILON
    B_normalizada = B_suavizada / B_suavizada.sum(axis=1, keepdims=True)

    # Actualizar A a partir de duraciones reales
    A_entrenada = np.zeros((n_estados, n_estados))
    for j in range(n_estados - 1):
        total = conteos_A_quedarse[j] + conteos_A_avanzar[j]
        if total > 0:
            A_entrenada[j, j]     = conteos_A_quedarse[j] / total
            A_entrenada[j, j + 1] = conteos_A_avanzar[j]  / total
        else:
            A_entrenada[j, j]     = 0.5
            A_entrenada[j, j + 1] = 0.5

    A_entrenada[n_estados - 1, n_estados - 1] = 1.0  # último estado absorbente

    modelos_hmm[palabra]["A"] = A_entrenada
    modelos_hmm[palabra]["B"] = B_normalizada

with open(FILE_MODELS_TRAINED, 'wb') as f:
    pickle.dump(modelos_hmm, f)
print(f"\n[✓] Modelos entrenados guardados en '{FILE_MODELS_TRAINED}'")

# ── Validación rápida ────────────────────────────────────────────────
ejemplo_p = "lift"
print(f"\n=== Verificación (ejemplo): {ejemplo_p.upper()} ===")
print("\nMatriz A entrenada:")
print(np.round(modelos_hmm[ejemplo_p]["A"], 4))
print(f"Suma filas A: {modelos_hmm[ejemplo_p]['A'].sum(axis=1)}")
min_prob = np.min(modelos_hmm[ejemplo_p]["B"])
print(f"\nProbabilidad mínima en B (debe ser > 0): {min_prob:.2e}")
print(f"Suma fila 0 de B (debe ser 1.0): {modelos_hmm[ejemplo_p]['B'][0].sum():.16f}")

print("\n" + "=" * 60)
print("SETUP COMPLETADO — Todos los archivos .pkl generados.")
print("=" * 60)
