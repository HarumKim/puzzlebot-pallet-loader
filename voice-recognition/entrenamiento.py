import pickle
import numpy as np

# Configuración
file_sequences = "secuencias_cuantizadas.pkl"
file_models_init = "modelos_hmm_iniciales.pkl"
file_models_trained = "modelos_hmm_entrenados.pkl"

epsilon = 1e-6
n_simbolos = 256

# Cargar las observaciones 'O' y los modelos iniciales
with open(file_sequences, 'rb') as f:
    dataset_secuencias = pickle.load(f)

with open(file_models_init, 'rb') as f:
    modelos_hmm = pickle.load(f)

print(f"{'='*60}")
print("Entrenamiento HMM: Segmentación Lineal y Suavizado")
print(f"{'='*60}\n")

for palabra, grabaciones in dataset_secuencias.items():
    print(f"-> Entrenando modelo para la palabra: '{palabra.upper()}'")
    
    n_estados = modelos_hmm[palabra]["n_estados"]
    
    # Matrices acumuladoras para los conteos directos
    conteos_B = np.zeros((n_estados, n_simbolos))
    conteos_A_quedarse = np.zeros(n_estados - 1)
    conteos_A_avanzar = np.zeros(n_estados - 1)

    # Separación de datos 80/20 (Hold-out)
    split = int(len(grabaciones) * 0.8)  # 80% entrenamiento, 20% prueba
    grabaciones_test = grabaciones[:split]     # Primeras 48
    
    # Procesar cada una de las 48 grabaciones
    for grabacion in grabaciones_test:
        O = grabacion["observacion_O"]
        T = len(O)
        
        # 1. Segmentación lineal (Partes iguales por estado)
        # Calculamos los índices de corte flotantes para no perder fracciones de frame
        limites_estados = np.linspace(0, T, n_estados + 1).astype(int)
        
        # Asignar cada frame a su respectivo estado asignado por tiempo
        for j in range(n_estados):
            inicio = limites_estados[j]
            fin = limites_estados[j+1]
            segmento_O = O[inicio:fin]
            
            # 2.Conteos para la matriz B (Emisiones)
            for simbolo in segmento_O:
                conteos_B[j, simbolo] += 1
                
            # 3.Conteos para la matriz A (Transiciones)
            # Solo para los estados que pueden avanzar (del 0 al penúltimo)
            if j < n_estados - 1:
                duracion_segmento = len(segmento_O)
                if duracion_segmento > 0:
                    # En un segmento de N frames, el modelo se "queda" N-1 veces y "avanza" 1 vez
                    conteos_A_quedarse[j] += max(0, duracion_segmento - 1)
                    conteos_A_avanzar[j] += 1

    # Actualización y suavizado de matriz B
    # Sumar epsilon obligatorio para evitar colapsos por observaciones no vistas
    B_suavizada = conteos_B + epsilon
    
    # Normalizar cada fila para que sumen exactamente 1.0 (Sin flotantes huérfanos)
    B_normalizada = B_suavizada / B_suavizada.sum(axis=1, keepdims=True)
    
    # Actualización de matriz A
    A_entrenada = np.zeros((n_estados, n_estados))
    for j in range(n_estados - 1):
        total_transiciones = conteos_A_quedarse[j] + conteos_A_avanzar[j]
        if total_transiciones > 0:
            a_jj = conteos_A_quedarse[j] / total_transiciones
            a_j_siguiente = conteos_A_avanzar[j] / total_transiciones
        else:
            # En caso extremo de un audio ultra corto, mantener el 50/50 inicial
            a_jj, a_j_siguiente = 0.5, 0.5
            
        A_entrenada[j, j] = a_jj
        A_entrenada[j, j+1] = a_j_siguiente
        
    A_entrenada[n_estados-1, n_estados-1] = 1.0 # El último estado sigue siendo absorbente

    # Guardar las matrices entrenadas en el diccionario del modelo
    modelos_hmm[palabra]["A"] = A_entrenada
    modelos_hmm[palabra]["B"] = B_normalizada

# Guardar los modelos robustos ya entrenados
with open(file_models_trained, 'wb') as f: 
    pickle.dump(modelos_hmm, f)

print(f"\n¡Entrenamiento completado! Modelos guardados en '{file_models_trained}'")

# Validación  
ejemplo_p = "lift"
print(f"\n=== Verificación (ejemplo): {ejemplo_p.upper()} ===")
print("\n1. Matriz A optimizada por duración real:")
print(modelos_hmm[ejemplo_p]["A"])
print(f"Suma filas Matriz A: {modelos_hmm[ejemplo_p]['A'].sum(axis=1)}")

print("\n2. Validación de Suavizado (Smoothing) en Matriz B:")
min_prob = np.min(modelos_hmm[ejemplo_p]["B"])
print(f"Probabilidad mínima encontrada en B (Debe ser > 0 debido a epsilon): {min_prob}")
print(f"Suma de una fila cualquiera de B (Debe ser exactamente 1.0): {np.sum(modelos_hmm[ejemplo_p]['B'][0])}")