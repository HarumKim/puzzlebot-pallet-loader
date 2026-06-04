"""
main.py
=======
Configuracion y ejecucion del Reconocedor de Palabras Aisladas (HMM + VQ).
 
Estructura de dataset esperada (una subcarpeta por palabra):
 
    DATAset/
        back/   alondra_back_01.wav  ...
        drop/   ...
        ...
 
Uso:
    pip install numpy librosa matplotlib soundfile
 
    # Entrenar, evaluar y GUARDAR el modelo (.pkl) en outputs/:
    python main.py
 
    # Reconocer un audio suelto usando el modelo ya guardado:
    python main.py --predict /ruta/a/un_audio.wav
 
    # Opcionales:
    python main.py --states 6
    python main.py --baum-welch
"""
 
import argparse
import reconocedor
 
# ----------------------------------------------------------------------
# Configuracion por defecto (puedes editar aqui en vez de usar la CLI)
# ----------------------------------------------------------------------
CONFIG = {
    "data_dir": "/home/alondra/Documents/DATASET/DATAset",
    # Audio / MFCC
    "sr": 16000, "n_mfcc": 13, "use_deltas": True,
    "win_ms": 25, "hop_ms": 10, "trim_db": 25,
    # HMM
    "n_states": 5,            # 4..8 (guia)
    # Estados por palabra segun complejidad fonetica, p.ej. {"lift": 7, "left": 7}
    "states_per_word": {},
    # Baum-Welch (OPCIONAL: activar solo tras validar el modelo de conteos)
    "baum_welch": False, "bw_iters": 10,
    # Particion
    "test_ratio": 0.3, "seed": 0,
    # Salidas
    "output_dir": "outputs",
    "model_file": "modelo_voz.pkl",   # se guarda dentro de output_dir
}
 
 
def main():
    ap = argparse.ArgumentParser(description="Reconocedor de palabras aisladas (HMM + VQ)")
    ap.add_argument("--data", default=CONFIG["data_dir"], help="carpeta del dataset")
    ap.add_argument("--states", type=int, default=CONFIG["n_states"], help="# de estados (4..8)")
    ap.add_argument("--baum-welch", action="store_true", help="activar Baum-Welch (opcional)")
    ap.add_argument("--test-ratio", type=float, default=CONFIG["test_ratio"])
    ap.add_argument("--no-deltas", action="store_true", help="usar solo MFCC base")
    ap.add_argument("--seed", type=int, default=CONFIG["seed"])
    ap.add_argument("--predict", metavar="WAV",
                    help="reconocer un .wav usando el modelo guardado (no entrena)")
    args = ap.parse_args()
 
    CONFIG["data_dir"] = args.data
    CONFIG["n_states"] = args.states
    CONFIG["baum_welch"] = args.baum_welch
    CONFIG["test_ratio"] = args.test_ratio
    CONFIG["use_deltas"] = not args.no_deltas
    CONFIG["seed"] = args.seed
 
    if args.predict:
        # Modo prediccion: carga el .pkl y clasifica un solo audio
        import os
        pkl = os.path.join(CONFIG["output_dir"], CONFIG["model_file"])
        bundle = reconocedor.load_model(pkl)
        palabra, scores = reconocedor.recognize_file(args.predict, bundle)
        print(f"\nPalabra reconocida: {palabra}")
        print("Log-likelihoods por modelo (mayor = mejor):")
        for w, s in sorted(scores.items(), key=lambda kv: kv[1], reverse=True):
            print(f"   {w:>8}: {s:.2f}")
    else:
        # Modo entrenamiento: entrena, evalua, dibuja y guarda el .pkl
        reconocedor.run(CONFIG)
 
 
if __name__ == "__main__":
    main()
 