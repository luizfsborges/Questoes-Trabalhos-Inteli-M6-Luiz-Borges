<img src="../assets/logo-inteli.png" alt="Logo do Inteli"/>

# Atividade 4: Backend para transmissão e armazenamento de imagens

## Enunciado

Desenvolva o software de um backend capaz de receber imagens e armazená-las adequadamente. Não há restrições com relação à tecnologia utilizada.

## Padrão de qualidade

Para esta atividade, espera-se a capacidade demonstrável de desenvolvimento de um backend capaz de interagir (não necessariamente em tempo real) com um sistema de visão computacional. A entrega deve ser um vídeo demonstrando o funcionamento do projeto, um texto conciso descrevendo como foi feita a implementação e o link para o repositório público no github onde foi feita a implementação.

1. Setup das rotas do backend. (peso 1)
2. Manipulação adequada da imagem de acordo com a estratégia escolhida (arquivos completos, frames) e integração com a rota do backend. (peso 2)
3. Armazenamento adequado da imagem. (peso 2)
4. Explicação coerente e concisa da implementação (min 250 caracteres e máximo 1500); (peso 3)
5. Congruência entre o que foi escrito e o código disposto no repositório do github; (peso 2)

## Demonstração do script desenvolvido 

https://github.com/luizfsborges/Questoes-Trabalhos-Inteli-M6-Luiz-Borges/assets/40524905/0bfce4e6-c75e-4670-8726-2dae89a6b5a0

## Explicação do script desenvolvido

O script usa as bibliotecas Flask, OpenCV e SQLLite para criar um aplicativo web capaz de receber uma imagem, ter pos´síveis rostos nela identificados e marcados com um retângulo vermelho, armazenar tal imagem em um banco de dados local e exibir a imagem com o retângulo vermelho na página web. 

A configuração do banco de dados é feita com o nome do arquivo do banco de dados definido como faces.db. A função create_database() é chamada para criar uma tabela chamada images com as colunas id, filename e image_data se ela não existir.

A função encontra_face(filename) é definida para detectar rostos em uma imagem. Ela usa o arquivo XML haarcascade_frontalface_default.xml (um classificador de detecção de faces pré-treinado) para detectar os rostos na imagem usando o OpenCV. Em seguida, desenha retângulos vermlhos ao redor dos rostos detectados na imagem e salva a imagem com os retângulos em um local temporário. O caminho para a imagem temporária é retornado.

A rota / é definida para a página inicial do aplicativo. 

A rota /upload é definida para receber uma imagem enviada por um formulário de upload. Se uma imagem for recebida, ela é salva em um diretório temporário no servidor. Em seguida, a função encontra_face() é chamada para detectar os rostos na imagem. A imagem é lida novamente, codificada em base64 e armazenada no banco de dados junto com o nome do arquivo (blob). Em seguida, o arquivo temporário é excluído.

A rota /temp_image é definida para exibir a imagem com os rostos detectados. Ela retorna o arquivo da imagem temporária como uma resposta para a solicitação HTTP.

