<img src="../assets/logo-inteli.png" alt="Logo do Inteli"/>

# Atividade 2: Simulação de robôs móveis com Gazebo

## Enunciado

Crie um pacote em ROS capaz de interagir com uma simulação feita no Gazebo de modo que a plataforma simulada do turtlebot3 seja capaz de mover-se de maneira controlada.
- Interagir com os tópicos e/ou serviços do turtlebot3 de modo a conseguir mandar comandos de velocidade e extrair dados de odometria.
- Conceber uma estrutura de dados capaz de armazenar a série de movimentos que devem ser feitos pelo robô para chegar no objetivo.
- Implementar uma rota pré-estabelecida

## Padrão de qualidade

Para esta atividade, espera-se a capacidade demonstrável de interagir com um ambiente de simulação de robôs, gerando um movimento controlado na plataforma turtlebot3. A entrega deve ser um vídeo demonstrando o funcionamento do projeto, um texto conciso descrevendo como foi feita a implementação e o link para o repositório público no github onde foi feita a implementação. O enunciado da atividade encontra-se no link anexado ao card.

Padrão de qualidade:

1. Setup adequado do ambiente de simulação; (peso 1)
2. Interação adequada com os tópicos relacionados ao robô simulado; (peso 2)
3. Demonstração de movimento controlado de acordo com uma rota pré-estabelecida; (peso 3)
4. Explicação coerente e concisa da implementação (min 250 caracteres e máximo 1500); (peso 2)
5. Congruência entre o que foi escrito e o código disposto no repositório do github; (peso 2)

## Desenvolvimento

No código fornecido, o Publisher é responsável por enviar comandos de velocidade linear para o tópico "/cmd_vel". Esses comandos são enviados periodicamente através do método publisher.publish(msg), onde msg contém as informações de velocidade a serem enviadas. O Gazebo, por sua vez, está configurado para receber essas mensagens do tópico "/cmd_vel" e interpretá-las para controlar o movimento do Turtlebot3 simulado. O Gazebo utiliza essas informações para atualizar a simulação do Turtlebot3, movendo-o para frente com a velocidade linear especificada.

Por outro lado, o Subscriber é responsável por receber informações de posição do Turtlebot3 simulado, que são enviadas pelo Gazebo para o tópico "/odom". Essas informações são recebidas pelo método callback_odometria(msg), onde msg contém os dados de posição atual do Turtlebot3. A partir dessas informações, o código calcula a distância percorrida pelo robô desde o início do movimento. Quando essa distância atinge ou ultrapassa a distância alvo especificada, o Turtlebot3 é parado por meio do método parar_turtlebot(). Dessa forma, o Publisher, o Subscriber e o Gazebo trabalham em conjunto para controlar o movimento do Turtlebot3 e monitorar sua posição durante a simulação.