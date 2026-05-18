import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.special import logsumexp
from sklearn.metrics import confusion_matrix, classification_report

# Configuracion
file_sequences      = "secuencias_cuantizadas.pkl"
file_models_trained = "modelos_hmm_entrenados.pkl"

with open(file_sequences, 'rb') as f:
    dataset_secuencias = pickle.load(f)
with open(file_models_trained, 'rb') as f:
    modelos_hmm = pickle.load(f)

# Versión vectorizada
def precompute_log_params(hmm):
    floor = np.finfo(float).tiny
    hmm["log_A"]  = np.log(np.maximum(hmm["A"],  floor))
    hmm["log_B"]  = np.log(np.maximum(hmm["B"],  floor))
    hmm["log_pi"] = np.log(np.maximum(hmm["pi"], floor))
    return hmm

for nombre in modelos_hmm:
    modelos_hmm[nombre] = precompute_log_params(modelos_hmm[nombre])

def forward_log_likelihood(O, HMM):
    log_alpha = HMM["log_pi"] + HMM["log_B"][:, O[0]]
    for t in range(1, len(O)):
        log_alpha = logsumexp(log_alpha[:, None] + HMM["log_A"], axis=0) \
                    + HMM["log_B"][:, O[t]]
    return logsumexp(log_alpha)

# ============================================================
print(f"{'='*60}")
print("  ENTREGABLES DE VERIFICACIÓN (CHECKLIST)")
print(f"{'='*60}\n")

# ENTREGABLE 1: SPARSITY EN B
ejemplo_p = "start"
hmm_ejemplo = modelos_hmm[ejemplo_p]
distribucion_estado_1 = hmm_ejemplo["B"][1]
indices_picos = np.where(distribucion_estado_1 > 0.01)[0]

print(f"1. [SPARSITY EN B] - Estado 1 para '{ejemplo_p.upper()}':")
print(f"   Índices con picos notables (>1% de prob.): {len(indices_picos)} de 256")
print(f"   Prob. acumulada en esos picos: {np.sum(distribucion_estado_1[indices_picos])*100:.2f}%")
print("   -> La mayoría de los 256 índices tienen valor cercano a epsilon.\n")

# ENTREGABLE 2: DIAGONALIDAD EN A
print(f"2. [DIAGONALIDAD EN A] - Progresión temporal para '{ejemplo_p.upper()}':")
print(np.round(hmm_ejemplo["A"], 4))
print("   -> Valores fuertes solo en diagonal (quedarse) y super-diagonal (avanzar).\n")

# ENTREGABLE 3: MATRIZ DE CONFUSIÓN
print("3. [MATRIZ DE CONFUSIÓN] - Evaluando únicamente el 20% de prueba...")

y_true = []
y_pred = []
palabras_lista = list(dataset_secuencias.keys())

for palabra_real, grabaciones in dataset_secuencias.items():
    # Mismo corte que el entrenamiento — solo grabaciones no vistas
    split = int(len(grabaciones) * 0.8)
    grabaciones_test = grabaciones[split:]          # grabaciones 48–59

    for grabacion in grabaciones_test:
        O = grabacion["observacion_O"]
        log_likelihoods  = {p: forward_log_likelihood(O, modelos_hmm[p])
                            for p in palabras_lista}
        palabra_ganadora = max(log_likelihoods, key=log_likelihoods.get)
        y_true.append(palabra_real)
        y_pred.append(palabra_ganadora)

cm = confusion_matrix(y_true, y_pred, labels=palabras_lista)

print("\n=== MATRIZ DE CONFUSIÓN (10×10) — Hold-out 20% ===")
print(f"{'':10}", "".join([f"{p[:5].upper():<7}" for p in palabras_lista]))
for i, palabra in enumerate(palabras_lista):
    print(f"{palabra[:9].upper():<10}", "".join([f"{v:<7}" for v in cm[i]]))

print("\n=== REPORTE DE CLASIFICACIÓN ===")
print(classification_report(y_true, y_pred, target_names=palabras_lista))

# GRÁFICAS
fig = plt.figure(figsize=(16, 6))
gs  = gridspec.GridSpec(1, 2, width_ratios=[1, 1.4])

# Gráfica 1 — Sparsity
ax1 = fig.add_subplot(gs[0])
colores = ['#7c3aed' if d > 0.01 else '#d1d5db' for d in distribucion_estado_1]
ax1.bar(range(256), distribucion_estado_1, color=colores, width=1.5)
ax1.set_title(f"Entregable 1: Sparsity en B\nEstado 1 — '{ejemplo_p.upper()}'",
              fontsize=12, fontweight='bold')
ax1.set_xlabel("Índice del Codebook (0–255)")
ax1.set_ylabel("Probabilidad")
ax1.legend(handles=[
    plt.Rectangle((0,0),1,1, color='#7c3aed', label=f'{len(indices_picos)} picos > 1%'),
    plt.Rectangle((0,0),1,1, color='#d1d5db', label=f'{256-len(indices_picos)} índices ≈ ε')
], loc='upper right', fontsize=9)
ax1.grid(True, alpha=0.3)

# Gráfica 2 — Matriz de confusión
ax2 = fig.add_subplot(gs[1])
im = ax2.imshow(cm, cmap='Blues', interpolation='nearest')
plt.colorbar(im, ax=ax2, shrink=0.85)
labels_upper = [p.upper() for p in palabras_lista]
ticks = np.arange(len(palabras_lista))
ax2.set_xticks(ticks); ax2.set_xticklabels(labels_upper, rotation=45, ha='right')
ax2.set_yticks(ticks); ax2.set_yticklabels(labels_upper)

# Anotar cada celda con su valor
thresh = cm.max() / 2
for i in range(len(palabras_lista)):
    for j in range(len(palabras_lista)):
        ax2.text(j, i, str(cm[i, j]),
                 ha='center', va='center', fontsize=9,
                 color='white' if cm[i, j] > thresh else 'black')

ax2.set_title("Entregable 3: Matriz de Confusión\nHold-out 20% (120 muestras no vistas)",
              fontsize=12, fontweight='bold')
ax2.set_xlabel("Predicción del Modelo")
ax2.set_ylabel("Clase Real")

plt.tight_layout()
plt.savefig("entregables_verificacion.png", dpi=150, bbox_inches='tight')
plt.show()
print("\n[✓] Gráfica guardada como 'entregables_verificacion.png'")