# Baseado em https://github.com/markwsilliman/turtlebot/blob/master/draw_a_square.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import radians
from time import sleep

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.move_turtle)
        self.move_forward = Twist() # responsável por fazer a tartagura ir para frente
        self.turn = Twist() # responsável por fazer a tartaruga virar

    def move_turtle(self):
        self.move_forward.linear.x = 0.2 # parâmetro do avanço da tartatura
        self.turn.linear.x = 0.0 # parâmetro nulo para o giro da tartaruga
        self.turn.angular.z = radians(45) # definição de grandeza angular para o giro da tartaruga

        # Laço de repetição que faz a tartagura avançar e virar, formando uma estrela
        while True:

            # Laço que faz a tartaruga avançar, por meio da publicação do comando de avanço
            print("Moving forward")
            for i in range(200):
                self.publisher_.publish(self.move_forward)
                sleep(0.1)
            
            # Laço que faz a tartaruga virar, por meio da publicação do comando de giro
            print("Turning")
            for j in range(30):
                self.publisher_.publish(self.turn)
                sleep(0.1)
         
# Função principal
def main(args=None):
    rclpy.init()
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()