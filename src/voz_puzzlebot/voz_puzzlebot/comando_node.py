"""
comando_node.py — Nodo ROS2 de reconocimiento de voz
Graba continuamente desde el micrófono, detecta silencio para segmentar
palabras y clasifica con HMM.

Tópicos publicados:
    /puzzlebot/command (std_msgs/String) — comando reconocido
    /voice/status      (std_msgs/String) — estado del sistema

Dependencias:
    pip install sounddevice librosa scipy scikit-learn numpy
"""

import os
import pickle
import threading
import numpy as np
import librosa
import sounddevice as sd

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from scipy.special import logsumexp


class VoiceCommandNode(Node):

    # ── Parámetros de audio ──────────────────────────────────────
    FS          = 16000
    N_MFCC      = 13
    HOP_LENGTH  = 512
    TOP_DB      = 20
    MIN_FRAMES  = 4

    # ── Detección de silencio ────────────────────────────────────
    RMS_SILENCIO    = 0.02  # umbral de energía para detectar voz
    DURACION_CHUNK  = 0.1    # segundos por chunk (~100ms)
    CHUNKS_SILENCIO = 6      # chunks silenciosos para cortar (~600ms)
    CHUNKS_VOZ_MIN  = 3      # chunks mínimos con voz para procesar (~300ms)

    # ── Rutas absolutas a los modelos ────────────────────────────
    BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
    FILE_CODEBOOK = os.path.join(BASE_DIR, "codebook_256.pkl")
    FILE_MODELOS  = os.path.join(BASE_DIR, "modelos_hmm_entrenados.pkl")

    def __init__(self):
        super().__init__('voice_command_node')

        # ── Cargar modelos ───────────────────────────────────────
        try:
            self.get_logger().info(f"Cargando modelos desde: {self.BASE_DIR}")
            with open(self.FILE_CODEBOOK, 'rb') as f:
                self.kmeans = pickle.load(f)
            with open(self.FILE_MODELOS, 'rb') as f:
                modelos_raw = pickle.load(f)
            self.modelos_hmm = {
                nombre: self._precompute_log_params(hmm)
                for nombre, hmm in modelos_raw.items()
            }
            self.get_logger().info(
                f"Modelos cargados: {list(self.modelos_hmm.keys())}"
            )
        except FileNotFoundError as e:
            self.get_logger().error(f"Archivo no encontrado: {e}")
            raise

        # ── Publicadores ─────────────────────────────────────────
        self.pub_command = self.create_publisher(String, '/puzzlebot/command', 10)
        self.pub_status  = self.create_publisher(String, '/voice/status',      10)

        # ── Estado interno ───────────────────────────────────────
        self._escuchando      = False
        self._buffer_chunks   = []
        self._pre_buffer      = []   # últimos chunks antes de detectar voz
        self._chunks_silencio = 0
        self._chunks_voz      = 0
        self._lock            = threading.Lock()
        self._hilo_grabacion  = None
        self._stop_grabacion  = threading.Event()

        self.get_logger().info("Nodo listo. Iniciando grabación continua.")
        self._iniciar_grabacion()

    def _iniciar_grabacion(self):
        self._escuchando = True
        self._stop_grabacion.clear()
        self._buffer_chunks   = []
        self._pre_buffer      = []
        self._chunks_silencio = 0
        self._chunks_voz      = 0

        self._hilo_grabacion = threading.Thread(
            target=self._loop_grabacion, daemon=True
        )
        self._hilo_grabacion.start()
        self._publicar_status("ESCUCHANDO")
        self.get_logger().info("🎤 Grabación iniciada.")

    def _detener_grabacion(self):
        self._escuchando = False
        self._stop_grabacion.set()
        self._publicar_status("PAUSADO")
        self.get_logger().info("Grabación detenida.")

    # LOOP DE GRABACIÓN — corre en hilo separado
    def _loop_grabacion(self):
        """
        Graba chunks de DURACION_CHUNK segundos continuamente.
        Detecta silencio para segmentar palabras y clasificar.
        """
        samples_por_chunk = int(self.FS * self.DURACION_CHUNK)

        self.get_logger().info("Hilo de grabación activo.")

        while not self._stop_grabacion.is_set():
            chunk = sd.rec(
                samples_por_chunk,
                samplerate=self.FS,
                channels=1,
                dtype='float32'
            )
            sd.wait()
            chunk = chunk.flatten()

            # Calcular energía del chunk
            rms     = float(np.sqrt(np.mean(chunk ** 2)))
            hay_voz = rms > self.RMS_SILENCIO

            with self._lock:
                if hay_voz:
                    if self._chunks_voz == 0 and self._pre_buffer:
                        # primer chunk con voz — prependear onset capturado
                        self._buffer_chunks.extend(self._pre_buffer)
                    self._pre_buffer      = []
                    self._buffer_chunks.append(chunk)
                    self._chunks_voz      += 1
                    self._chunks_silencio  = 0

                else:
                    if self._chunks_voz > 0:
                        self._chunks_silencio += 1

                        if self._chunks_silencio >= self.CHUNKS_SILENCIO:
                            # Silencio suficiente → clasificar si hay voz acumulada
                            if self._chunks_voz >= self.CHUNKS_VOZ_MIN:
                                audio = np.concatenate(self._buffer_chunks)
                                threading.Thread(
                                    target=self._clasificar,
                                    args=(audio,),
                                    daemon=True
                                ).start()

                            # Limpiar buffer
                            self._buffer_chunks   = []
                            self._pre_buffer      = []
                            self._chunks_silencio = 0
                            self._chunks_voz      = 0
                    else:
                        # acumular pre-roll: máximo 2 chunks (200ms antes de voz)
                        self._pre_buffer.append(chunk)
                        if len(self._pre_buffer) > 2:
                            self._pre_buffer.pop(0)

        self.get_logger().info("Hilo de grabación terminado.")

    # PIPELINE DE CLASIFICACIÓN
    def _clasificar(self, audio: np.ndarray):
        try:
            # 1. Trimming
            audio_trim, _ = librosa.effects.trim(audio, top_db=self.TOP_DB)
            if len(audio_trim) < self.HOP_LENGTH * self.MIN_FRAMES:
                self.get_logger().warn("Audio demasiado corto, descartado.")
                return

            # 2. MFCC + delta + delta-delta
            mfcc   = librosa.feature.mfcc(
                y=audio_trim, sr=self.FS,
                n_mfcc=self.N_MFCC, hop_length=self.HOP_LENGTH
            )
            delta1 = librosa.feature.delta(mfcc)
            delta2 = librosa.feature.delta(mfcc, order=2)
            mfcc   = np.vstack([mfcc, delta1, delta2]).T  # (n_frames, N_MFCC*3)

            # 3. Cuantización vectorial
            O = self.kmeans.predict(mfcc).tolist()

            # 4. Forward sobre los 10 modelos
            scores  = {
                nombre: self._forward(O, hmm)
                for nombre, hmm in self.modelos_hmm.items()
            }
            palabra = max(scores, key=scores.get)

            # 5. Confianza mínima — descartar si el margen es muy pequeño
            top2   = sorted(scores.values(), reverse=True)[:2]
            margen = top2[0] - top2[1]
            if margen < 3.0:
                self.get_logger().warn(
                    f"Clasificación poco confiable "
                    f"('{palabra.upper()}', margen={margen:.2f}) — descartado."
                )
                return

            # 6. Publicar resultado
            msg      = String()
            msg.data = palabra
            self.pub_command.publish(msg)
            self._publicar_status(f"COMANDO: {palabra.upper()}")
            self.get_logger().info(
                f"Comando: '{palabra.upper()}' "
                f"(score: {scores[palabra]:.4f}, margen: {margen:.2f})"
            )

        except Exception as e:
            self.get_logger().error(f"Error en clasificación: {e}")

    # UTILIDADES
    @staticmethod
    def _precompute_log_params(hmm: dict) -> dict:
        floor = np.finfo(float).tiny
        hmm["log_A"]  = np.log(np.maximum(hmm["A"],  floor))
        hmm["log_B"]  = np.log(np.maximum(hmm["B"],  floor))
        hmm["log_pi"] = np.log(np.maximum(hmm["pi"], floor))
        return hmm

    def _forward(self, O: list, HMM: dict) -> float:
        log_alpha = HMM["log_pi"] + HMM["log_B"][:, O[0]]
        for t in range(1, len(O)):
            log_alpha = logsumexp(log_alpha[:, None] + HMM["log_A"], axis=0) \
                        + HMM["log_B"][:, O[t]]
        return float(logsumexp(log_alpha))

    def _publicar_status(self, texto: str):
        msg      = String()
        msg.data = texto
        self.pub_status.publish(msg)

    def destroy_node(self):
        self._detener_grabacion()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()