"""
fsm_control_node.py — Mux de seguridad por voz para el Puzzlebot autónomo

Estados:
    IDLE     → esperando comando START
    RUNNING  → misión activa, pasa /nav/cmd_vel a /cmd_vel
    STOP     → parada de seguridad, bloquea navegación
    LIFT     → detenido ejecutando recogida de pallet
    DROP     → detenido ejecutando depósito de pallet

Comandos de voz:
    stop  → parada de seguridad inmediata
    start → reanudar misión autónoma
    lift  → recoger pallet
    drop  → depositar pallet

Tópicos suscritos:
    /puzzlebot/command  (std_msgs/String)      — comando reconocido por HMM
    /nav/cmd_vel        (geometry_msgs/Twist)  — velocidad del nodo de navegación

Tópicos publicados:
    /cmd_vel            (geometry_msgs/Twist)  — velocidad final al robot
    /fsm/state          (std_msgs/String)      — estado actual
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class FSMControlNode(Node):

    # ── Comandos válidos ─────────────────────────────────────────
    COMANDOS = {"stop", "start", "lift", "drop", "forward"}

    # ── Definición de estados ────────────────────────────────────
    ESTADO_IDLE    = "idle"
    ESTADO_RUNNING = "running"
    ESTADO_STOP    = "stop"
    ESTADO_LIFT    = "lift"
    ESTADO_DROP    = "drop"

    def __init__(self):
        super().__init__('fsm_control_node')

        self._estado          = self.ESTADO_RUNNING
        self._mission_active  = True
        self._nav_twist       = Twist()

        # ── Publicadores ─────────────────────────────────────────
        self.pub_cmd_vel = self.create_publisher(Twist,  '/cmd_vel',   10)
        self.pub_state   = self.create_publisher(String, '/fsm/state', 10)

        # ── Suscripciones ────────────────────────────────────────
        self.create_subscription(
            String, '/puzzlebot/command', self._comando_callback, 10
        )
        self.create_subscription(
            Twist, '/nav/cmd_vel', self._nav_callback, 10
        )

        # Timer 10 Hz — publica /cmd_vel continuamente
        self.create_timer(0.1, self._publicar_cmd_vel)

        self.get_logger().info(
            "FSM Control Node listo.\n"
            "  Estado inicial: RUNNING\n"
            "  Comandos: STOP | START | LIFT | DROP"
        )
        self._publicar_estado()

    # ════════════════════════════════════════════════════════════
    # NAVEGACIÓN — pasa a /cmd_vel solo si misión activa
    # ════════════════════════════════════════════════════════════
    def _nav_callback(self, msg: Twist):
        if self._mission_active:
            self._nav_twist = msg
            self.pub_cmd_vel.publish(msg)  # reenvío inmediato, sin esperar el timer

    # ════════════════════════════════════════════════════════════
    # COMANDO DE VOZ — máquina de estados
    # ════════════════════════════════════════════════════════════
    def _comando_callback(self, msg: String):
        comando = msg.data.strip().lower()

        if comando not in self.COMANDOS:
            self.get_logger().warn(
                f"'{comando.upper()}' no reconocido — ignorado."
            )
            return

        self.get_logger().info(f"Comando: '{comando.upper()}'")

        if comando == "stop":
            self._ejecutar_stop()
        elif comando in ("start", "forward"):
            self._ejecutar_start()
        elif comando == "lift":
            self._ejecutar_lift()
        elif comando == "drop":
            self._ejecutar_drop()

    # ════════════════════════════════════════════════════════════
    # ACCIONES POR ESTADO
    # ════════════════════════════════════════════════════════════
    def _ejecutar_stop(self):
        """Parada de seguridad — bloquea navegación inmediatamente."""
        self._mission_active = False
        self._nav_twist      = Twist()
        self.pub_cmd_vel.publish(Twist())
        self._transicion(self.ESTADO_STOP)

    def _ejecutar_start(self):
        """Reanudar misión — habilita paso de /nav/cmd_vel."""
        self._mission_active = True
        self._transicion(self.ESTADO_RUNNING)

    def _ejecutar_lift(self):
        """Recoger pallet — detiene motores y ejecuta acción de recogida."""
        self._mission_active = False
        self._nav_twist      = Twist()
        self.pub_cmd_vel.publish(Twist())
        self._transicion(self.ESTADO_LIFT)
        self.get_logger().info(
            "Ejecutando LIFT — agrega aquí la lógica del actuador."
        )

    def _ejecutar_drop(self):
        """Depositar pallet — detiene motores y ejecuta acción de depósito."""
        self._mission_active = False
        self._nav_twist      = Twist()
        self.pub_cmd_vel.publish(Twist())
        self._transicion(self.ESTADO_DROP)
        self.get_logger().info(
            "Ejecutando DROP — agrega aquí la lógica del actuador."
        )

    # ════════════════════════════════════════════════════════════
    # PUBLICAR /cmd_vel CONTINUAMENTE (10 Hz)
    # ════════════════════════════════════════════════════════════
    def _publicar_cmd_vel(self):
        if self._mission_active:
            self.pub_cmd_vel.publish(self._nav_twist)
        else:
            self.pub_cmd_vel.publish(Twist())

    # ════════════════════════════════════════════════════════════
    # TRANSICIÓN DE ESTADO
    # ════════════════════════════════════════════════════════════
    def _transicion(self, nuevo_estado: str):
        if nuevo_estado != self._estado:
            self.get_logger().info(
                f"Estado: {self._estado.upper()} → {nuevo_estado.upper()}"
            )
            self._estado = nuevo_estado
            self._publicar_estado()

    def _publicar_estado(self):
        msg      = String()
        msg.data = self._estado.upper()
        self.pub_state.publish(msg)

    # ════════════════════════════════════════════════════════════
    # CIERRE LIMPIO
    # ════════════════════════════════════════════════════════════
    def destroy_node(self):
        self.get_logger().info("Deteniendo robot antes de cerrar.")
        self.pub_cmd_vel.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FSMControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
