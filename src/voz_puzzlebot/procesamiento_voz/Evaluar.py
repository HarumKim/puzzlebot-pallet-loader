"""
Evaluar.py — Evaluación completa del sistema HMM
Requiere haber ejecutado setup.py previamente.

Secciones:
    1. Clasificación Hold-out 20%  (Clasificación.py)
    2. Entregables de verificación (Evaluacion.py)
       - Sparsity en B
       - Diagonalidad en A
       - Matriz de confusión 10×10 con gráficas

Archivos requeridos:
    secuencias_cuantizadas.pkl
    modelos_hmm_entrenados.pkl

Archivos generados:
    entregables_verificacion.png

Dependencias:
    pip install scipy scikit-learn matplotlib numpy
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.special import logsumexp
from sklearn.metrics import confusion_matrix, classification_report
from collections import defaultdict

# CONFIGURACIÓN
FILE_SEQUENCES      = "secuencias_cuantizadas.pkl"
FILE_MODELS_TRAINED = "modelos_hmm_entrenados.pkl"
EJEMPLO_PALABRA     = "start"   # Palabra usada para mostrar Sparsity y Diagonalidad

# ── Cargar datos ─────────────────────────────────────────────────
with open(FILE_SEQUENCES, 'rb') as f:
    dataset_secuencias = pickle.load(f)
with open(FILE_MODELS_TRAINED, 'rb') as f:
    modelos_hmm = pickle.load(f)

# ── Pre-calcular parámetros logarítmicos (una sola vez) ──────────
def precompute_log_params(hmm: dict) -> dict:
    floor = np.finfo(float).tiny
    hmm["log_A"]  = np.log(np.maximum(hmm["A"],  floor))
    hmm["log_B"]  = np.log(np.maximum(hmm["B"],  floor))
    hmm["log_pi"] = np.log(np.maximum(hmm["pi"], floor))
    return hmm

for nombre in modelos_hmm:
    modelos_hmm[nombre] = precompute_log_params(modelos_hmm[nombre])

# ── Algoritmo Forward vectorizado ────────────────────────────────
def forward_log_likelihood(O: list, HMM: dict) -> float:
    log_alpha = HMM["log_pi"] + HMM["log_B"][:, O[0]]
    for t in range(1, len(O)):
        log_alpha = logsumexp(log_alpha[:, None] + HMM["log_A"], axis=0) \
                    + HMM["log_B"][:, O[t]]
    return float(logsumexp(log_alpha))

palabras_lista = list(dataset_secuencias.keys())

# SECCIÓN 1 — CLASIFICACIÓN HOLD-OUT 20%
print("=" * 60)
print("SECCIÓN 1: Clasificación — Hold-out 20% (datos no vistos)")
print("=" * 60 + "\n")

total    = 0
aciertos = 0
matriz_confusion_raw = defaultdict(lambda: defaultdict(int))

for palabra_real, grabaciones in dataset_secuencias.items():
    split = int(len(grabaciones) * 0.8)
    grabaciones_test = grabaciones[split:]   # grabaciones 48–59

    for grabacion in grabaciones_test:
        O = grabacion["observacion_O"]
        resultados = {
            nombre: forward_log_likelihood(O, hmm)
            for nombre, hmm in modelos_hmm.items()
        }
        palabra_predicha = max(resultados, key=resultados.get)
        matriz_confusion_raw[palabra_real][palabra_predicha] += 1

        if palabra_real == palabra_predicha:
            aciertos += 1
        total += 1

print(f"Aciertos: {aciertos} / {total}")
print(f"Precisión global: {(aciertos / total) * 100:.2f}%\n")

# Matriz de confusión en texto
print(f"{'Matriz de Confusión':^60}")
print(f"{'(filas=real, columnas=predicho)':^60}\n")
header = f"{'':>10}" + "".join(f"{p[:5]:>7}" for p in palabras_lista)
print(header)
print("-" * len(header))
for palabra_real in palabras_lista:
    fila = f"{palabra_real:>10}"
    for palabra_pred in palabras_lista:
        fila += f"{matriz_confusion_raw[palabra_real][palabra_pred]:>7}"
    print(fila)

# SECCIÓN 2 — ENTREGABLES DE VERIFICACIÓN (CHECKLIST)
print("\n" + "=" * 60)
print("SECCIÓN 2: Entregables de Verificación (Checklist)")
print("=" * 60 + "\n")

hmm_ejemplo = modelos_hmm[EJEMPLO_PALABRA]

# ── Entregable 1: Sparsity en B ──────────────────────────────────
distribucion_estado_1 = hmm_ejemplo["B"][1]
indices_picos = np.where(distribucion_estado_1 > 0.01)[0]

print(f"1. [SPARSITY EN B] — Estado 1 para '{EJEMPLO_PALABRA.upper()}':")
print(f"   Índices con picos notables (>1% de prob.): {len(indices_picos)} de 256")
print(f"   Prob. acumulada en esos picos: {np.sum(distribucion_estado_1[indices_picos]) * 100:.2f}%")
print("   -> La mayoría de los 256 índices tienen valor cercano a epsilon.\n")

# ── Entregable 2: Diagonalidad en A ──────────────────────────────
print(f"2. [DIAGONALIDAD EN A] — Progresión temporal para '{EJEMPLO_PALABRA.upper()}':")
print(np.round(hmm_ejemplo["A"], 4))
print("   -> Valores fuertes solo en diagonal (quedarse) y super-diagonal (avanzar).\n")

# ── Entregable 3: Matriz de confusión con sklearn ─────────────────
print("3. [MATRIZ DE CONFUSIÓN] — Hold-out 20% con reporte detallado:")

y_true, y_pred = [], []
for palabra_real, grabaciones in dataset_secuencias.items():
    split = int(len(grabaciones) * 0.8)
    for grabacion in grabaciones[split:]:
        O = grabacion["observacion_O"]
        scores = {p: forward_log_likelihood(O, modelos_hmm[p]) for p in palabras_lista}
        y_true.append(palabra_real)
        y_pred.append(max(scores, key=scores.get))

cm = confusion_matrix(y_true, y_pred, labels=palabras_lista)

print("\n=== MATRIZ DE CONFUSIÓN (10×10) — Hold-out 20% ===")
print(f"{'':10}", "".join([f"{p[:5].upper():<7}" for p in palabras_lista]))
for i, palabra in enumerate(palabras_lista):
    print(f"{palabra[:9].upper():<10}", "".join([f"{v:<7}" for v in cm[i]]))

print("\n=== REPORTE DE CLASIFICACIÓN ===")
print(classification_report(y_true, y_pred, target_names=palabras_lista))

# SECCIÓN 3 — GRÁFICAS
fig = plt.figure(figsize=(16, 6))
gs  = gridspec.GridSpec(1, 2, width_ratios=[1, 1.4])

# Gráfica 1 — Sparsity en B
ax1 = fig.add_subplot(gs[0])
colores = ['#7c3aed' if d > 0.01 else '#d1d5db' for d in distribucion_estado_1]
ax1.bar(range(256), distribucion_estado_1, color=colores, width=1.5)
ax1.set_title(f"Entregable 1: Sparsity en B\nEstado 1 — '{EJEMPLO_PALABRA.upper()}'",
              fontsize=12, fontweight='bold')
ax1.set_xlabel("Índice del Codebook (0–255)")
ax1.set_ylabel("Probabilidad")
ax1.legend(handles=[
    plt.Rectangle((0, 0), 1, 1, color='#7c3aed', label=f'{len(indices_picos)} picos > 1%'),
    plt.Rectangle((0, 0), 1, 1, color='#d1d5db', label=f'{256 - len(indices_picos)} índices ≈ ε')
], loc='upper right', fontsize=9)
ax1.grid(True, alpha=0.3)

# Gráfica 2 — Matriz de confusión
ax2 = fig.add_subplot(gs[1])
im  = ax2.imshow(cm, cmap='Blues', interpolation='nearest')
plt.colorbar(im, ax=ax2, shrink=0.85)
labels_upper = [p.upper() for p in palabras_lista]
ticks = np.arange(len(palabras_lista))
ax2.set_xticks(ticks); ax2.set_xticklabels(labels_upper, rotation=45, ha='right')
ax2.set_yticks(ticks); ax2.set_yticklabels(labels_upper)

thresh = cm.max() / 2
for i in range(len(palabras_lista)):
    for j in range(len(palabras_lista)):
        ax2.text(j, i, str(cm[i, j]), ha='center', va='center', fontsize=9,
                 color='white' if cm[i, j] > thresh else 'black')

ax2.set_title("Entregable 3: Matriz de Confusión\nHold-out 20% (120 muestras no vistas)",
              fontsize=12, fontweight='bold')
ax2.set_xlabel("Predicción del Modelo")
ax2.set_ylabel("Clase Real")

plt.tight_layout()
plt.savefig("entregables_verificacion.png", dpi=150, bbox_inches='tight')
plt.show()
print("[✓] Gráfica guardada como 'entregables_verificacion.png'")
