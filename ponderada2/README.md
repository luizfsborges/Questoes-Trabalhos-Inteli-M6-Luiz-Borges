<img src="../assets/logo-inteli.png" alt="Logo do Inteli"/>

# Atividade 2: Otimização de rotas no Gazebo

## Enunciado

Crie um pacote em ROS capaz de interagir com uma simulação feita no Gazebo de modo que a plataforma simulada do turtlebot3 seja capaz de navegar pelo labirinto. Para isso, será necessário:
- Interagir com os tópicos e/ou serviços do turtlebot3 de modo a conseguir mandar comandos de velocidade e extrair dados de odometria.
- Conceber uma estrutura de dados capaz de armazenar a série de movimentos que devem ser feitos pelo robô para chegar no objetivo.
- Representar o labirinto utilizando um grafo.
- Escolher e implementar um algoritmo de otimização de rota baseado em grafos.

O robô deve partir de uma das pontas do labirinto e chegar à ponta diametralmente oposta, como ilustrado na figura abaixo:

![Mundo labirinto no gazebo](./assets/mundo.png)

## Padrão de qualidade

Para esta atividade, espera-se a capacidade demonstrável de utilizar grafos para a representação de ambientes físicos e implementar algoritmos clássicos para otimização de rota. A entrega deve ser um vídeo demonstrando o funcionamento do projeto, um texto conciso descrevendo como foi feita a implementação e o link para o repositório público no github onde foi feita a implementação. O enunciado da atividade encontra-se no link anexado ao card.

Padrão de qualidade:

1. Representação fidedigna do ambiente físico escolhido como um grafo; (peso 1)
2. Escolha adequada do algoritmo de grafo para otimização da rota e demonstrar que compreendeu o algoritmo; (peso 2)
3. Funcionamento correto do software capaz de gerar a rota otimizada utilizando conceitos de programação orientada à objetos; (peso 3)
4. Explicação coerente e concisa da implementação (min 250 caracteres e máximo 1500); (peso 2)
5. Congruência entre o que foi escrito e o código disposto no repositório do github; (peso 2)