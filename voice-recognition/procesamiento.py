import os
import librosa
import numpy as np
import pickle
from sklearn.cluster import KMeans

# --- CONFIGURACIÓN ---
fs = 16000
n_mfcc = 13          # Número estándar de coeficientes MFCC
n_clusters = 256     # Tamaño del Codebook solicitado por el profesor
base_dir = "DATASET"
output_codebook = "codebook_256.pkl"
output_sequences = "secuencias_cuantizadas.pkl"

print("Fase 1: Extrayendo MFCC de todas las grabaciones...")

todos_los_frames_mfcc = []  # Para entrenar el Codebook global
dataset_estructurado = {}   # Para guardar las secuencias finales

# Recorrer cada palabra (clase) en el dataset
for word in os.listdir(base_dir):
    word_dir = os.path.join(base_dir, word)
    if not os.path.isdir(word_dir):
        continue
    
    dataset_estructurado[word] = []
    print(f"-> Procesando palabra: '{word.upper()}'")
    
    for filename in os.listdir(word_dir):
        if not filename.endswith(".wav"):
            continue
            
        path = os.path.join(word_dir, filename)
        
        # 1. Cargar audio
        y, sr = librosa.load(path, sr=fs)
        
        # 2. Preprocesamiento: Remover silencios al inicio y final (Trimming)
        y_trimmed, _ = librosa.effects.trim(y, top_db=20)
        
        # 3. Extracción de MFCC
        # hop_length=512 a 16kHz da ventanas de ~32ms
        mfcc = librosa.feature.mfcc(y=y_trimmed, sr=sr, n_mfcc=n_mfcc, hop_length=512)
        
        # Transponer para tener los frames como filas (Formato: [n_frames, n_mfcc])
        mfcc_frames = mfcc.T
        
        # Guardar en la lista global para el Codebook
        todos_los_frames_mfcc.append(mfcc_frames)
        
        # Guardar temporalmente para la cuantización posterior
        dataset_estructurado[word].append({
            "filename": filename,
            "mfcc": mfcc_frames
        })

# Concatenar todos los vectores de todos los audios para entrenar K-Means
X_train = np.vstack(todos_los_frames_mfcc)
print(f"\nTotal de vectores (frames) para entrenar el Codebook: {X_train.shape[0]}")

print(f"\nFase 2: Entrenando el Codebook de {n_clusters} vectores con K-Means...")
kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
kmeans.fit(X_train)

# Guardar el modelo del Codebook (servirá para el nodo de ROS2 en tiempo real)
with open(output_codebook, 'wb') as f:
    pickle.dump(kmeans, f)
print(f"¡Codebook guardado con éxito en '{output_codebook}'!")

print("\nFase 3: Cuantizando audios a secuencias de enteros (0-255)...")
secuencias_finales = {}

for word, grabaciones in dataset_estructurado.items():
    secuencias_finales[word] = []
    print(f"-> Cuantizando palabra: '{word.upper()}'")
    
    for grabacion in grabaciones:
        # Mapear cada frame de MFCC al índice del centroide más cercano (0 a 255)
        secuencia_enteros = kmeans.predict(grabacion["mfcc"])
        
        # Convertir a lista para que se vea como el ejemplo de la guía
        secuencia_lista = secuencia_enteros.tolist()
        
        secuencias_finales[word].append({
            "archivo": grabacion["filename"],
            "observacion_O": secuencia_lista
        })

# Guardar las secuencias cuantizadas para la entrega y la fase HMM
with open(output_sequences, 'wb') as f:
    pickle.dump(secuencias_finales, f)

print(f"\n{'='*50}")
print("¡PROCESO COMPLETADO CON ÉXITO!")
print(f"Archivo de secuencias listo: '{output_sequences}'")
print("{'='*50}")

# --- EJEMPLO DE SALIDA EN PANTALLA ---
palabra_ejemplo = list(secuencias_finales.keys())[0]
ejemplo = secuencias_finales[palabra_ejemplo][0]
print(f"\nEjemplo de representación para '{palabra_ejemplo.upper()}' ({ejemplo['archivo']}):")
print(f"O = {ejemplo['observacion_O'][:15]}... (Longitud total: {len(ejemplo['observacion_O'])} frames)")