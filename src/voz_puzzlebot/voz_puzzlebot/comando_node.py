"""
comando_node.py — Nodo ROS2 de reconocimiento de voz (HMM + VQ)
 
Graba continuamente desde el microfono, segmenta por silencio y clasifica con
el modelo entrenado. Usa EXACTAMENTE la misma extraccion de MFCC y la misma
cuantizacion vectorial del entrenamiento, porque ambas viven dentro del
'modelo_voz.pkl' (se cargan via reconocedor.load_model / recognize_array).
 
Topicos publicados:
    /puzzlebot/command (std_msgs/String) — comando reconocido
    /voice/status      (std_msgs/String) — estado del sistema
 
IMPORTANTE:
  - 'reconocedor.py' debe estar en la MISMA carpeta que este nodo (se importa y,
    ademas, es necesario para des-serializar el .pkl, que guarda referencias a
    las clases DiscreteHMM y VectorQuantizer).
  - Copia el 'modelo_voz.pkl' generado por el entrenamiento junto a este archivo
    (o dentro de una subcarpeta 'outputs/').
 
Dependencias del robot:
    pip install sounddevice librosa numpy
"""
import os
import sys
import threading
import numpy as np
import sounddevice as sd
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
# Asegura que 'reconocedor.py' (misma carpeta) sea importable, tanto para usar
# sus funciones como para que pickle pueda reconstruir las clases del modelo.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import reconocedor
 
 
class VoiceCommandNode(Node):
 
    # ── Deteccion de silencio / segmentacion ─────────────────────
    FS              = 16000   # debe coincidir con el sr del entrenamiento
    RMS_SILENCIO    = 0.02    # umbral de energia para detectar voz
    DURACION_CHUNK  = 0.1     # segundos por chunk (~100 ms)
    CHUNKS_SILENCIO = 6       # chunks silenciosos para cortar (~600 ms)
    CHUNKS_VOZ_MIN  = 3       # chunks minimos con voz para procesar (~300 ms)
 
    # ── Confianza ────────────────────────────────────────────────
    # Margen minimo (log-likelihood top1 - top2) para aceptar. Ajustalo segun
    # tus pruebas: subelo si acepta ruido/comandos fuera de vocabulario.
    MARGEN_MIN = 5.0
 
    # ── Ruta al modelo entrenado ─────────────────────────────────
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
 
    def __init__(self):
        super().__init__('voice_command_node')
 
        # ── Cargar el modelo (codebook + 10 HMM + params de MFCC) ─
        pkl_path = self._buscar_modelo()
        try:
            self.get_logger().info(f"Cargando modelo: {pkl_path}")
            self.bundle = reconocedor.load_model(pkl_path)
            self.get_logger().info(
                f"Modelo cargado. Palabras: {self.bundle['words']}"
            )
        except FileNotFoundError:
            self.get_logger().error(
                f"No se encontro el modelo. Copia 'modelo_voz.pkl' junto al nodo "
                f"o en {os.path.join(self.BASE_DIR, 'outputs')}."
            )
            raise
 
        # ── Publicadores ─────────────────────────────────────────
        self.pub_command = self.create_publisher(String, '/puzzlebot/command', 10)
        self.pub_status  = self.create_publisher(String, '/voice/status',      10)

        # ── Suscripcion a activacion web ─────────────────────────
        self.create_subscription(String, '/voice/command', self._web_command_callback, 10)

        # ── Estado interno ───────────────────────────────────────
        self._active          = False
        self._buffer_chunks   = []
        self._pre_buffer      = []
        self._chunks_silencio = 0
        self._chunks_voz      = 0
        self._lock            = threading.Lock()
        self._stop_grabacion  = threading.Event()

        self.get_logger().info("Nodo listo. Esperando activacion desde la web.")
        self._publicar_status("ESPERANDO")
 
    def _buscar_modelo(self):
        """Busca modelo_voz.pkl junto al nodo o en ./outputs/."""
        candidatos = [
            os.path.join(self.BASE_DIR, "modelo_voz.pkl"),
            os.path.join(self.BASE_DIR, "outputs", "modelo_voz.pkl"),
        ]
        for c in candidatos:
            if os.path.exists(c):
                return c
        return candidatos[0]   # devuelve el primero para que el error sea claro
 
    # ── Comando desde la pagina web ──────────────────────────────
    def _web_command_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self._iniciar_grabacion()
        elif cmd == 'stop':
            self._detener_grabacion()

    # ── Control de grabacion ─────────────────────────────────────
    def _iniciar_grabacion(self):
        if self._active:
            return
        self._active = True
        self._stop_grabacion.clear()
        self._buffer_chunks   = []
        self._pre_buffer      = []
        self._chunks_silencio = 0
        self._chunks_voz      = 0
        if hasattr(self, '_hilo_grabacion') and self._hilo_grabacion is not None:
            self._hilo_grabacion.join(timeout=1.0)
        self._hilo_grabacion = threading.Thread(
            target=self._loop_grabacion, daemon=True
        )
        self._hilo_grabacion.start()
        self._publicar_status("ESCUCHANDO")
        self.get_logger().info("Grabacion iniciada.")

    def _detener_grabacion(self):
        if not self._active:
            return
        self._active = False
        self._stop_grabacion.set()
        sd.stop()
        self._publicar_status("PAUSADO")
        self.get_logger().info("Grabacion detenida.")
 
    # ── LOOP DE GRABACION (hilo separado) ────────────────────────
    def _loop_grabacion(self):
        samples_por_chunk = int(self.FS * self.DURACION_CHUNK)
        self.get_logger().info("Hilo de grabacion activo.")
 
        while not self._stop_grabacion.is_set():
            chunk = sd.rec(samples_por_chunk, samplerate=self.FS,
                           channels=1, dtype='float32')
            try:
                sd.wait()
            except Exception:
                break
            chunk = chunk.flatten()
 
            rms     = float(np.sqrt(np.mean(chunk ** 2)))
            hay_voz = rms > self.RMS_SILENCIO
 
            with self._lock:
                if hay_voz:
                    if self._chunks_voz == 0 and self._pre_buffer:
                        self._buffer_chunks.extend(self._pre_buffer)  # onset
                    self._pre_buffer      = []
                    self._buffer_chunks.append(chunk)
                    self._chunks_voz      += 1
                    self._chunks_silencio  = 0
                else:
                    if self._chunks_voz > 0:
                        self._chunks_silencio += 1
                        if self._chunks_silencio >= self.CHUNKS_SILENCIO:
                            if self._chunks_voz >= self.CHUNKS_VOZ_MIN:
                                audio = np.concatenate(self._buffer_chunks)
                                threading.Thread(target=self._clasificar,
                                                 args=(audio,), daemon=True).start()
                            self._buffer_chunks   = []
                            self._pre_buffer      = []
                            self._chunks_silencio = 0
                            self._chunks_voz      = 0
                    else:
                        self._pre_buffer.append(chunk)   # pre-roll (~200 ms)
                        if len(self._pre_buffer) > 2:
                            self._pre_buffer.pop(0)
 
        self.get_logger().info("Hilo de grabacion terminado.")
 
    # ── CLASIFICACION ────────────────────────────────────────────
    def _clasificar(self, audio: np.ndarray):
        try:
            # Toda la cadena MFCC -> VQ -> Forward(log) ocurre aqui, identica
            # al entrenamiento, porque usa los parametros guardados en el bundle.
            palabra, scores = reconocedor.recognize_array(audio, self.FS, self.bundle)
 
            if palabra is None:
                self.get_logger().warn("Audio demasiado corto, descartado.")
                return
 
            ordenados = sorted(scores.values(), reverse=True)
            margen = ordenados[0] - ordenados[1] if len(ordenados) > 1 else float("inf")
            if margen < self.MARGEN_MIN:
                self.get_logger().warn(
                    f"Poco confiable ('{palabra}', margen={margen:.2f}) — descartado."
                )
                return
 
            msg = String(); msg.data = palabra
            self.pub_command.publish(msg)
            self._publicar_status(f"COMANDO: {palabra.upper()}")
            self.get_logger().info(
                f"Comando: '{palabra.upper()}' "
                f"(score={scores[palabra]:.2f}, margen={margen:.2f})"
            )
        except Exception as e:
            self.get_logger().error(f"Error en clasificacion: {e}")
 
    # ── UTILIDADES ───────────────────────────────────────────────
    def _publicar_status(self, texto: str):
        msg = String(); msg.data = texto
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