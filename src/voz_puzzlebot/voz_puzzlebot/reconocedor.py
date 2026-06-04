"""
reconocedor.py
==============
Reconocedor de Palabras Aisladas (HMM + VQ).
 
Restriccion de librerias:
  - librosa: SOLO para calcular los MFCC (permitido por la guia).
  - numpy:   base numerica (librosa ya lo requiere).
  - matplotlib: SOLO para dibujar los 3 entregables de verificacion;
                no interviene en ningun algoritmo.
 
TODO lo demas es propio (sin scipy ni sklearn):
  - Cuantizacion Vectorial: k-means++ y asignacion al vecino mas cercano.
  - HMM discreto Bakis: init por conteos, Forward en log (logsumexp propio),
    Baum-Welch opcional.
 
Instalar:  pip install numpy librosa matplotlib soundfile
"""
 
import os
import glob
import pickle
import numpy as np
import librosa
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
 
EPS = 1e-6            # epsilon de suavizado (guia)
CODEBOOK_SIZE = 256   # tamano del codebook (guia)
 
 
# ====================================================================== #
#  Utilidad propia:  logsumexp estable  (reemplaza scipy.special.logsumexp)
# ====================================================================== #
def logsumexp(a, axis=None):
    """log(sum(exp(a))) estable. Soporta axis=None (escalar) y axis=0/1."""
    a = np.asarray(a, dtype=float)
    amax = np.max(a, axis=axis, keepdims=True)
    amax = np.where(np.isfinite(amax), amax, 0.0)   # evita (-inf) - (-inf)
    with np.errstate(divide="ignore"):
        res = np.log(np.sum(np.exp(a - amax), axis=axis, keepdims=True)) + amax
    if axis is None:
        return res.item()
    return np.squeeze(res, axis=axis)
 
 
# ====================================================================== #
#  1) CARACTERISTICAS:  MFCC (librosa)  +  VQ propia (k-means a mano)
# ====================================================================== #
def _mfcc_core(y, sr, n_mfcc, use_deltas, win_ms, hop_ms, trim_db):
    """Calcula MFCC (+deltas) a partir de una senal y ya cargada a 'sr'."""
    y = np.asarray(y, dtype=float)
    if trim_db is not None:
        y, _ = librosa.effects.trim(y, top_db=trim_db)   # recorta silencios
    n_fft = int(sr * win_ms / 1000)
    if len(y) < n_fft:
        y = np.pad(y, (0, n_fft))
    hop = int(sr * hop_ms / 1000)
    mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=n_mfcc, n_fft=n_fft, hop_length=hop)
    if not use_deltas:
        return mfcc.T
    if mfcc.shape[1] < 9:    # delta necesita >=9 tramas; rellena por borde si falta
        mfcc = np.pad(mfcc, ((0, 0), (0, 9 - mfcc.shape[1])), mode="edge")
    feats = [mfcc, librosa.feature.delta(mfcc), librosa.feature.delta(mfcc, order=2)]
    return np.vstack(feats).T
 
 
def extract_mfcc(path, sr=16000, n_mfcc=13, use_deltas=True,
                 win_ms=25, hop_ms=10, trim_db=25):
    """Devuelve matriz (T, D) de MFCC desde un archivo. D = 3*n_mfcc con deltas."""
    y, _ = librosa.load(path, sr=sr)
    return _mfcc_core(y, sr, n_mfcc, use_deltas, win_ms, hop_ms, trim_db)
 
 
def _kmeanspp_init(X, k, rng):
    """Inicializacion k-means++ (mejores semillas que el azar puro)."""
    n = X.shape[0]
    centers = np.empty((k, X.shape[1]))
    centers[0] = X[rng.integers(n)]
    d2 = ((X - centers[0]) ** 2).sum(1)
    for i in range(1, k):
        probs = d2 / d2.sum()
        centers[i] = X[rng.choice(n, p=probs)]
        d2 = np.minimum(d2, ((X - centers[i]) ** 2).sum(1))
    return centers
 
 
class VectorQuantizer:
    """Codebook de 256 vectores. K-means propio. Se ajusta SOLO con train."""
 
    def __init__(self, k=CODEBOOK_SIZE, seed=0, n_iter=30):
        self.k = k
        self.seed = seed
        self.n_iter = n_iter
 
    def _dist2(self, Xs):
        """Distancias^2 (n,k) via ||x||^2 - 2x.c + ||c||^2 (sin gastar memoria)."""
        x2 = (Xs ** 2).sum(1)[:, None]
        c2 = (self.centers ** 2).sum(1)[None, :]
        return x2 - 2.0 * Xs @ self.centers.T + c2
 
    def fit(self, feat_list):
        X = np.vstack(feat_list)
        # estandarizacion z-score (propia) para que ninguna dim domine la distancia
        self.mean = X.mean(0)
        self.std = X.std(0) + 1e-8
        Xs = (X - self.mean) / self.std
 
        rng = np.random.default_rng(self.seed)
        self.centers = _kmeanspp_init(Xs, self.k, rng)
        for _ in range(self.n_iter):
            labels = self._dist2(Xs).argmin(1)
            new = self.centers.copy()
            for j in range(self.k):
                m = labels == j
                if m.any():
                    new[j] = Xs[m].mean(0)
                else:
                    new[j] = Xs[rng.integers(Xs.shape[0])]   # reinicia vacio
            if np.allclose(new, self.centers):
                self.centers = new
                break
            self.centers = new
        return self
 
    def quantize(self, feat):
        """Matriz (T, D)  ->  secuencia de enteros 0..255."""
        Xs = (feat - self.mean) / self.std
        return self._dist2(Xs).argmin(1).astype(int)
 
 
# ====================================================================== #
#  2) HMM DISCRETO PROPIO  (topologia Bakis / Left-to-Right)
# ====================================================================== #
class DiscreteHMM:
    def __init__(self, n_states, n_symbols=CODEBOOK_SIZE, eps=EPS):
        self.N = int(n_states)
        self.M = int(n_symbols)
        self.eps = float(eps)
        self.pi = np.zeros(self.N); self.pi[0] = 1.0   # Left-to-Right: arranca en 0
        self.A = np.zeros((self.N, self.N))
        self.B = np.zeros((self.N, self.M))
 
    # ---- Entrenamiento por conteos (segmentacion lineal) ----
    def init_by_counting(self, sequences):
        N, M = self.N, self.M
        counts = np.zeros((N, M))
        durations = [[] for _ in range(N)]
        for O in sequences:
            O = np.asarray(O, dtype=int)
            T = len(O)
            if T == 0:
                continue
            # frame t -> estado floor(t*N/T): cada ~100/N % de la senal por estado
            state_of_frame = np.clip((np.arange(T) * N) // T, 0, N - 1)
            for s in range(N):
                idx = np.where(state_of_frame == s)[0]
                if len(idx):
                    durations[s].append(len(idx))
                    vals, c = np.unique(O[idx], return_counts=True)
                    counts[s, vals] += c
 
        # Matriz B + suavizado epsilon + renormalizar a suma EXACTA 1
        B = counts.copy()
        rs = B.sum(axis=1, keepdims=True)
        empty = rs[:, 0] == 0
        B[~empty] /= rs[~empty]
        B[empty] = 1.0 / M
        B += self.eps
        B /= B.sum(axis=1, keepdims=True)
        self.B = B
 
        # Matriz A: si dura d frames, a_ii = 1 - 1/d, a_i,i+1 = 1/d
        A = np.zeros((N, N))
        for s in range(N):
            d = max(np.mean(durations[s]) if durations[s] else 1.0, 1.0)
            if s < N - 1:
                A[s, s] = 1.0 - 1.0 / d
                A[s, s + 1] = 1.0 / d
            else:
                A[s, s] = 1.0   # ultimo estado absorbente
        self.A = A
        return self
 
    # ---- Forward en log (reconocimiento) ----
    def log_likelihood(self, O):
        O = np.asarray(O, dtype=int)
        with np.errstate(divide="ignore"):
            logA, logB, logpi = np.log(self.A), np.log(self.B), np.log(self.pi)
        log_alpha = logpi + logB[:, O[0]]
        for t in range(1, len(O)):
            log_alpha = logsumexp(log_alpha[:, None] + logA, axis=0) + logB[:, O[t]]
        return logsumexp(log_alpha)
 
    # ---- Baum-Welch (refinamiento OPCIONAL; respeta estructura Bakis) ----
    def _fwd(self, O, logA, logB):
        T = len(O); la = np.full((T, self.N), -np.inf)
        with np.errstate(divide="ignore"):
            la[0] = np.log(self.pi) + logB[:, O[0]]
        for t in range(1, T):
            la[t] = logsumexp(la[t - 1][:, None] + logA, axis=0) + logB[:, O[t]]
        return la
 
    def _bwd(self, O, logA, logB):
        T = len(O); lb = np.full((T, self.N), -np.inf); lb[T - 1] = 0.0
        for t in range(T - 2, -1, -1):
            lb[t] = logsumexp(logA + logB[:, O[t + 1]][None, :] + lb[t + 1][None, :], axis=1)
        return lb
 
    def baum_welch(self, sequences, n_iter=10, tol=1e-3, verbose=False):
        seqs = [np.asarray(O, int) for O in sequences if len(O) > 0]
        allowed = self.A > 0
        prev = -np.inf
        for it in range(n_iter):
            with np.errstate(divide="ignore"):
                logA, logB = np.log(self.A), np.log(self.B)
            A_num = np.zeros((self.N, self.N)); A_den = np.zeros(self.N)
            B_num = np.zeros((self.N, self.M)); B_den = np.zeros(self.N)
            total = 0.0
            for O in seqs:
                T = len(O)
                la, lb = self._fwd(O, logA, logB), self._bwd(O, logA, logB)
                ll = logsumexp(la[T - 1]); total += ll
                gamma = np.exp(la + lb - ll)
                for t in range(T - 1):
                    xi = np.exp(la[t][:, None] + logA
                                + logB[:, O[t + 1]][None, :] + lb[t + 1][None, :] - ll)
                    A_num += xi; A_den += gamma[t]
                for t in range(T):
                    B_num[:, O[t]] += gamma[t]
                B_den += gamma.sum(axis=0)
            # re-estimar A (solo entradas permitidas por la estructura Bakis)
            A_new = np.zeros_like(self.A)
            for i in range(self.N):
                if A_den[i] > 0:
                    A_new[i] = A_num[i] / A_den[i]
            A_new *= allowed
            rs = A_new.sum(axis=1, keepdims=True); rs[rs == 0] = 1.0
            A_new /= rs
            if A_new[self.N - 1].sum() == 0:
                A_new[self.N - 1, self.N - 1] = 1.0
            self.A = A_new
            # re-estimar B + suavizado
            B_new = np.zeros_like(self.B)
            for j in range(self.N):
                B_new[j] = B_num[j] / B_den[j] if B_den[j] > 0 else 1.0 / self.M
            B_new += self.eps
            B_new /= B_new.sum(axis=1, keepdims=True)
            self.B = B_new
            if verbose:
                print(f"    [BW] iter {it+1}: logL = {total:.2f}")
            if abs(total - prev) < tol:
                break
            prev = total
        return self
 
 
# ====================================================================== #
#  3) PIPELINE  (carga, entrena, reconoce, verifica)
# ====================================================================== #
def load_dataset(data_dir):
    """Devuelve {palabra: [rutas .wav]}. Una subcarpeta por palabra."""
    words = sorted(d for d in os.listdir(data_dir)
                   if os.path.isdir(os.path.join(data_dir, d)))
    data = {}
    for w in words:
        wavs = sorted(glob.glob(os.path.join(data_dir, w, "*.wav")))
        if wavs:
            data[w] = wavs
    return data
 
 
def _split(paths, ratio, rng):
    paths = list(paths); rng.shuffle(paths)
    n_test = max(1, int(round(len(paths) * ratio)))
    return paths[n_test:], paths[:n_test]
 
 
def run(cfg):
    rng = np.random.default_rng(cfg["seed"])
    os.makedirs(cfg["output_dir"], exist_ok=True)
 
    data = load_dataset(cfg["data_dir"])
    words = list(data.keys())
    print(f"Palabras encontradas ({len(words)}): {words}")
    if not words:
        raise SystemExit(f"No hay palabras en '{cfg['data_dir']}'. "
                         "Revisa la ruta y que haya una subcarpeta por palabra.")
 
    # MFCC + particion
    tr_feats, te_feats = {}, {}
    for w in words:
        tr, te = _split(data[w], cfg["test_ratio"], rng)
        ex = lambda p: extract_mfcc(p, cfg["sr"], cfg["n_mfcc"], cfg["use_deltas"],
                                    cfg["win_ms"], cfg["hop_ms"], cfg["trim_db"])
        tr_feats[w] = [ex(p) for p in tr]
        te_feats[w] = [ex(p) for p in te]
        print(f"  {w}: {len(tr)} train, {len(te)} test")
 
    # Codebook VQ propio (solo train)
    print("Construyendo codebook VQ (256) con k-means propio...")
    vq = VectorQuantizer(CODEBOOK_SIZE, cfg["seed"]).fit(
        [f for w in words for f in tr_feats[w]])
 
    tr_seq = {w: [vq.quantize(f) for f in tr_feats[w]] for w in words}
    te_seq = {w: [vq.quantize(f) for f in te_feats[w]] for w in words}
    print("Ejemplo secuencia VQ ('%s'): %s ..." %
          (words[0], tr_seq[words[0]][0][:12].tolist()))
 
    # Entrenar un HMM por palabra
    models = {}
    for w in words:
        n_states = cfg["states_per_word"].get(w, cfg["n_states"])
        m = DiscreteHMM(n_states).init_by_counting(tr_seq[w])
        if cfg["baum_welch"]:
            print(f"  Baum-Welch en '{w}'...")
            m.baum_welch(tr_seq[w], n_iter=cfg["bw_iters"])
        models[w] = m
    print(f"Entrenados {len(models)} modelos (Baum-Welch={'ON' if cfg['baum_welch'] else 'OFF'}).")
 
    # Reconocimiento (Forward en log; gana el mayor log-likelihood)
    y_true, y_pred = [], []
    for w in words:
        for O in te_seq[w]:
            scores = {ww: models[ww].log_likelihood(O) for ww in words}
            y_true.append(w); y_pred.append(max(scores, key=scores.get))
    acc = np.mean([t == p for t, p in zip(y_true, y_pred)])
    n_ok = sum(t == p for t, p in zip(y_true, y_pred))
    print(f"\nExactitud en prueba: {acc*100:.1f}%  ({n_ok}/{len(y_true)})")
 
    # Entregables de verificacion
    _verify_B(models[words[0]], words[0], cfg["output_dir"])
    _verify_A(models[words[0]], words[0], cfg["output_dir"])
    _confusion(y_true, y_pred, words, cfg["output_dir"])
 
    # Guardar todo (codebook + 10 HMM + parametros) en un solo .pkl
    bundle = {
        "words": words,
        "vq": vq,
        "models": models,
        # parametros de MFCC, para que la prediccion use EXACTAMENTE los mismos
        "feat": {k: cfg[k] for k in
                 ("sr", "n_mfcc", "use_deltas", "win_ms", "hop_ms", "trim_db")},
    }
    pkl_path = os.path.join(cfg["output_dir"], cfg.get("model_file", "modelo_voz.pkl"))
    with open(pkl_path, "wb") as f:
        pickle.dump(bundle, f)
    print(f"\nModelo guardado en: {pkl_path}")
 
    return models, vq, (y_true, y_pred), acc
 
 
# ====================================================================== #
#  4) CARGA Y PREDICCION  (para usar el modelo ya entrenado, p.ej. en ROS)
# ====================================================================== #
def load_model(pkl_path):
    """Carga el bundle (.pkl) generado por run()."""
    with open(pkl_path, "rb") as f:
        return pickle.load(f)
 
 
def recognize_file(wav_path, bundle):
    """
    Reconoce un .wav suelto con un modelo ya cargado.
    Devuelve (palabra_predicha, dict_de_log_likelihoods).
    """
    feat = extract_mfcc(wav_path, **bundle["feat"])
    O = bundle["vq"].quantize(feat)
    scores = {w: m.log_likelihood(O) for w, m in bundle["models"].items()}
    return max(scores, key=scores.get), scores
 
 
def recognize_array(y, in_sr, bundle, min_frames=8):
    """
    Reconoce una senal de audio EN MEMORIA (array de numpy), usando EXACTAMENTE
    los mismos parametros de MFCC y la misma VQ del entrenamiento (vienen en el
    bundle). Util para nodos en vivo (ROS) que capturan a un buffer.
 
    Devuelve (palabra | None, dict_de_log_likelihoods). None si el audio es
    demasiado corto.
    """
    fc = bundle["feat"]
    sr = fc["sr"]
    y = np.asarray(y, dtype=float)
    if in_sr != sr:                       # remuestrea si llega a otra frecuencia
        y = librosa.resample(y, orig_sr=in_sr, target_sr=sr)
    n_fft = int(sr * fc["win_ms"] / 1000)
    hop = int(sr * fc["hop_ms"] / 1000)
    if len(y) < n_fft + min_frames * hop:   # claramente muy corto -> no clasificar
        return None, {}
    feat = _mfcc_core(y, sr, fc["n_mfcc"], fc["use_deltas"],
                      fc["win_ms"], fc["hop_ms"], fc["trim_db"])
    if feat.shape[0] < min_frames:
        return None, {}
    O = bundle["vq"].quantize(feat)
    scores = {w: m.log_likelihood(O) for w, m in bundle["models"].items()}
    return max(scores, key=scores.get), scores
 
 
# ---- Entregable 1: Sparsity en B (estado 0) ----
def _verify_B(model, word, out_dir):
    b0 = model.B[0]
    print(f"\n[Sparsity B, '{word}' estado 0] top-10 indices:")
    for i in np.argsort(b0)[::-1][:10]:
        print(f"   indice {int(i):3d}: {b0[i]:.4f}")
    print(f"   indices con prob < 1e-3: {np.mean(b0 < 1e-3)*100:.1f}%")
    plt.figure(figsize=(10, 3.2))
    plt.bar(range(model.M), b0, width=1.0)
    plt.title(f"Sparsity de B - '{word}', Estado 0")
    plt.xlabel("Indice del codebook (0..255)"); plt.ylabel("Probabilidad")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "B_sparsity_estado0.png"), dpi=130); plt.close()
 
 
# ---- Entregable 2: Diagonalidad en A ----
def _verify_A(model, word, out_dir):
    np.set_printoptions(precision=3, suppress=True)
    print(f"\n[Diagonalidad A, '{word}']:\n{model.A}")
    plt.figure(figsize=(4.5, 4))
    plt.imshow(model.A, cmap="viridis", vmin=0, vmax=1)
    for i in range(model.N):
        for j in range(model.N):
            if model.A[i, j] > 0:
                plt.text(j, i, f"{model.A[i, j]:.2f}", ha="center", va="center",
                         color="white", fontsize=8)
    plt.colorbar(label="Probabilidad"); plt.title(f"Matriz A (Bakis) - '{word}'")
    plt.xlabel("Estado destino"); plt.ylabel("Estado origen"); plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "A_diagonalidad.png"), dpi=130); plt.close()
 
 
# ---- Entregable 3: Matriz de confusion ----
def _confusion(y_true, y_pred, words, out_dir):
    idx = {w: i for i, w in enumerate(words)}; n = len(words)
    cm = np.zeros((n, n), int)
    for t, p in zip(y_true, y_pred):
        cm[idx[t], idx[p]] += 1
    print("\n[Matriz de confusion] (filas=real, columnas=predicho)")
    print("        " + " ".join(f"{w[:5]:>6}" for w in words))
    for i, w in enumerate(words):
        print(f"{w[:7]:>7} " + " ".join(f"{cm[i,j]:6d}" for j in range(n)))
    plt.figure(figsize=(7, 6))
    plt.imshow(cm, cmap="Blues"); plt.colorbar(label="conteo")
    plt.xticks(range(n), words, rotation=45, ha="right"); plt.yticks(range(n), words)
    thr = cm.max() / 2 if cm.max() else 0
    for i in range(n):
        for j in range(n):
            plt.text(j, i, cm[i, j], ha="center", va="center",
                     color="white" if cm[i, j] > thr else "black", fontsize=9)
    plt.ylabel("Palabra real"); plt.xlabel("Palabra reconocida")
    plt.title("Matriz de confusion"); plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "matriz_confusion.png"), dpi=130); plt.close()
    return cm
 