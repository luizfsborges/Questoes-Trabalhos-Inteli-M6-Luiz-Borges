#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt

class Controladorturtlebot(Node):
    def __init__(self):
        super().__init__('turtlebot_pose_listener')
        # Criação do publisher e do subscriber
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic="/cmd_vel",
            qos_profile=10
        )
        self.subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.callback_odometria,
            qos_profile=10
        )
        self.subscriber  # para sumir o aviso de variável não utilizada

    # Função de callback que será chamada sempre que uma mensagem for recebida para executar o loop de controle do turtlebot
    def callback_odometria(self, msg):
        if self.distancia_alvo == 0.0:
            return

        pose = msg.pose.pose
        posicao = pose.position

        self.posicao_atual = posicao

        if self.posicao_inicial is None:
            self.posicao_inicial = posicao

        distancia = self.calcular_distancia(self.posicao_inicial, self.posicao_atual)

        # Se a distância percorrida for maior ou igual a distância alvo, o turtlebot para
        if distancia >= self.distancia_alvo:
            self.parar_turtlebot()
            self.distancia_alvo = 0.0

    # Função que faz o turtlebot se mover por uma distância informada
    def percorre_caminho(self, distancia_alvo):
        self.distancia_alvo = distancia_alvo
        self.posicao_inicial = None
        self.posicao_atual = None
        self.mover_para_frente()

    # Função para movimentar o turtlebot para frente zerando sua velocidade angular
    def mover_para_frente(self):
        msg = Twist()
        msg.linear.x = 0.5  # velocidade linear
        msg.angular.z = 0.0  # velocidade angular
        self.publisher.publish(msg)
        self.get_logger().info("Andando pra frente")

    # Função para parar o turtlebot zerando suas velocidades linear e angular
    def parar_turtlebot(self):
        msg = Twist()
        msg.linear.x = 0.0  # velocidade linear
        msg.angular.z = 0.0  # velocidade angular
        self.publisher.publish(msg)
        self.get_logger().info("Turtlebot parou")

    # Função para calcular a distância entre duas posições (x, y)
    def calcular_distancia(self, posicao1, posicao2):
        return sqrt((posicao2.x - posicao1.x) ** 2 + (posicao2.y - posicao1.y) ** 2)

# Função principal
def main(args=None):
    rclpy.init(args=args)
    controlador = Controladorturtlebot()
    # Faz a turtlebot se mover por uma distância de 5 unidades
    controlador.percorre_caminho(7)
    rclpy.spin(controlador)
    controlador.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()