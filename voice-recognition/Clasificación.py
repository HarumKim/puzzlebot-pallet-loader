import pickle
import numpy as np
from scipy.special import logsumexp
from collections import defaultdict

file_sequences    = "secuencias_cuantizadas.pkl"
file_models_trained = "modelos_hmm_entrenados.pkl"

with open(file_sequences, 'rb') as f:
    dataset_secuencias = pickle.load(f)
with open(file_models_trained, 'rb') as f:
    modelos_hmm = pickle.load(f)

# Pre-calcular parámetros logarítmicos
def precompute_log_params(hmm):
    floor = np.finfo(float).tiny
    hmm["log_A"]  = np.log(np.maximum(hmm["A"],  floor))
    hmm["log_B"]  = np.log(np.maximum(hmm["B"],  floor))
    hmm["log_pi"] = np.log(np.maximum(hmm["pi"], floor))
    return hmm

for nombre in modelos_hmm:
    modelos_hmm[nombre] = precompute_log_params(modelos_hmm[nombre])

def forward_log_likelihood(O, HMM):
    log_A  = HMM["log_A"]
    log_B  = HMM["log_B"]
    log_pi = HMM["log_pi"]
    log_alpha = log_pi + log_B[:, O[0]]
    for t in range(1, len(O)):
        log_alpha = logsumexp(log_alpha[:, None] + log_A, axis=0) + log_B[:, O[t]]
    return logsumexp(log_alpha)

# Evaluación sobre el 20% no visto 
print(f"{'='*60}")
print("Evaluación con Hold-out 20% (datos no vistos en entrenamiento)")
print(f"{'='*60}\n")

total = 0
aciertos = 0
palabras = list(dataset_secuencias.keys())
matriz_confusion = defaultdict(lambda: defaultdict(int))

for palabra_real, grabaciones in dataset_secuencias.items():

    # El mismo corte que se usó al entrenar
    split = int(len(grabaciones) * 0.8)
    grabaciones_test = grabaciones[split:]   # grabaciones 48–59

    for grabacion in grabaciones_test:
        O = grabacion["observacion_O"]

        resultados = {
            nombre: forward_log_likelihood(O, hmm)
            for nombre, hmm in modelos_hmm.items()
        }
        palabra_predicha = max(resultados, key=resultados.get)
        matriz_confusion[palabra_real][palabra_predicha] += 1

        if palabra_real == palabra_predicha:
            aciertos += 1
        total += 1

# Resultados globales 
print(f"Aciertos: {aciertos} / {total}")
print(f"Precisión global: {(aciertos/total)*100:.2f}%\n")

# Matriz de confusión 
print(f"{'Matriz de Confusión':^60}")
print(f"{'(filas=real, columnas=predicho)':^60}\n")

header = f"{'':>10}" + "".join(f"{p[:5]:>7}" for p in palabras)
print(header)
print("-" * len(header))

for palabra_real in palabras:
    fila = f"{palabra_real:>10}"
    for palabra_pred in palabras:
        count = matriz_confusion[palabra_real][palabra_pred]
        fila += f"{count:>7}"
    print(fila)