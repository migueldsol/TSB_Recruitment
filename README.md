# Recruitment _Project_ Tecnico Solar Boat

Development of a global planer and a program that converts a png image into a data structure for TSB_Recruitment 2022/2023

## Electric Systems - Global Planner

## Objective

Create, in ros, a global planner node that is able to trace the shortest path between two given coordinates and publish it to a topic. Also it was given a nautical Chart with the purpose of transforming it into a data structure for the global planner.

The Nautical Chart

![](/TSB/CartaNautica.png)

Example of Data Structure

![](/TSB/DataStructure.png)

Dentro da pasta [TSB](/TSB) temos duas pastas. Na pasta [images](/data/images) temos as imagens que foram usadas para treinar a rede neuronal e na pasta [labels](/data/labels) temos o ficheiro classes.txt (com as labels de cada um dos postes) e temos um ficheiro para cada imagem que contem o número da label e as coordenadas de cada uma das caixas da label. Dentro dessa pasta está também o ficheiro [dataset.yml](/data/dataset.yml) que vai ser usado pelo YOLOv5.

- **Exemplo:** (Vision00126.jpg)  
  ![](/misc/Vision00126.png)

  - Vision00126.txt:

    - 1 0.984688 0.495556 0.022187 0.665000
    - 3 0.288906 0.491389 0.021250 0.751667
    - 0 0.604141 0.493889 0.025156 0.718889
    - 1 0.437734 0.496111 0.017656 0.702778
    - 0 0.811867 0.495139 0.025328 0.702500

  - 0, 1, 2 e 3 corresponde a RedMarker, DangerMarker, ShallowMarker e GreenMarker, respetivamente, e os números correspondem aos cantos das caixas.

## Resultados

Imagens após serem detetados os objetos usando a rede neuronal:

![](/misc/1.png)
![](/misc/4.png)
![](/misc/2.png)
![](/misc/3.png)

## Simulação

![](/misc/Complexas.gif)

## Análise

Como podemos ver ao longo dos epochs a precisão e o recall vai melhorando.

**Precision** - mede o quão precisas são as medições.

- Formula:

![](/misc/precision.png)

**Recall** - mede o quão bom se encontra todos os casos positivos

- Formula:

  ![](/misc/recall.png)

**mAP** - mean average precision

![](/misc/graph.png)

Fonte: [Dados do Gráfico](/misc/Dados_Grafico.md) (também estão aqui todos os dados obtidos: [resultados.csv](/misc/results.csv))

## Algoritmo de deteção de objetos

[YOLOv5](https://github.com/ultralytics/yolov5)

YOLOv5 é baseado no algoritmo YOLO e significa 'You only look once'. Decidi usar este modelo porque é um dos mais rápidos, mais precisos e um dos melhores para deteção em tempo real.

A rede neuronal foi treinado por 1000 Epochs (número de vezes que o algoritmo passa por todo o dataset), com um batch size de 64 e uma resolução de imagem de 640p.

## Servidor usado para treinar a rede neuronal

[Google Colab](https://research.google.com/colaboratory/)

Usei o Google Colab devido a este estar online 24/7 e possuir GPUs bastante rápidas (NVIDIA Tesla K80).

Tem também o benefício de, por causa de usar estas GPU, puder treinar a rede neuronal de uma forma mais precisa devido à elevada VRAM que elas possuem (houve alturas em que foi usado mais de 12GB de VRAM uma só vez).

Problemas:

- Devido às limitações de utilização do Colab, tive de treinar a rede neuronal ao longo de dias e através de múltiplas contas da Google, de forma a conseguir treinar a rede neuronal enquanto algumas contas estavam banidas de usar GPUs.
- Devido a ser um servidor temporario todos os dados são apagados quando o servidor é reiniciado. Uma forma de manter os dados é escrever diretamente no Google Drive e desta forma podemos também treinar com várias contas usando apenas uma pasta partilhada.

## Annotation Tool Software usado para marcar as imagens para serem usadas para o treino

[Computer Vision Annotation Tool (CVAT)](https://cvat.org/)

Utilizei este software devido a ser online e puder anotar em qualquer lado.

Usei cerca de 1/3 das imagens fornecidas para puder ter imagens para testar e devido a algumas imagens não serem úteis para o treino da rede neuronal.

## Machine Learning Framework

[PyTorch](https://pytorch.org/)
