import pickle
import numpy as np

# Configuración de la topología
palabras = ["start", "stop", "lift", "drop", "left", "right", "forward", "back", "fast", "slow"]
n_estados = 5      # Elegido entre el rango de 4 a 8 pedido por el profesor
n_simbolos = 256   # Tamaño de tu Codebook de la entrega anterior

modelos_hmm = {}

print("Inicializando Topología Bakis (Left-to-Right) para los 10 modelos...")

for palabra in palabras:
    # 1. Inicializar Matriz de Transición A (Restricción Bakis)
    A = np.zeros((n_estados, n_estados))
    for i in range(n_estados - 1):
        # Inicialización estándar equilibrada: 50% quedarse, 50% avanzar
        A[i, i] = 0.5
        A[i, i+1] = 0.5
    A[n_estados-1, n_estados-1] = 1.0 # El último estado es absorbente
    
    # 2. Inicializar Matriz de Emisión B (Distribución uniforme para los 256 símbolos)
    # Cada estado tiene la misma probabilidad (1/256) de emitir cualquier símbolo al inicio
    B = np.full((n_estados, n_simbolos), 1.0 / n_simbolos)
    
    # 3. Inicializar Vector de Probabilidad Inicial Pi
    # En un modelo Left-to-Right estricto, SIEMPRE se empieza en el Estado 0
    pi = np.zeros(n_estados)
    pi[0] = 1.0
    
    # Guardar los parámetros para esta palabra
    modelos_hmm[palabra] = {
        "A": A,
        "B": B,
        "pi": pi,
        "n_estados": n_estados
    }

# Guardar la estructura inicializada
output_file = "modelos_hmm_iniciales.pkl"
with open(output_file, 'wb') as f:
    pickle.dump(modelos_hmm, f)

print(f"\n¡Estructura de modelos completada con éxito y guardada en '{output_file}'!")

# Validación y ejemplo 
ejemplo_palabra = "start"
print(f"\n=== Verificación de matriz A para '{ejemplo_palabra.upper()}' ===")
print(modelos_hmm[ejemplo_palabra]["A"])
print(f"\nSuma de filas de la matriz A (deben ser 1.0): {modelos_hmm[ejemplo_palabra]['A'].sum(axis=1)}")
print(f"Dimensión de la matriz de emisión B: {modelos_hmm[ejemplo_palabra]['B'].shape}")