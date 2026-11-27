# Desafio de navegação autônoma: mapeamento do labirinto

Um robô num labirinto 29×29 que **não conhece o mapa**. Ele só enxerga as 8 células ao redor. O programa `navegador.py` faz o robô:

1. **mapear** o labirinto inteiro, explorando-o sozinho;
2. **voltar ao início** e seguir o **menor caminho** até o alvo, usando o mapa que construiu.

O simulador (jogo, serviços e sensores) é o repositório `culling_games`, do professor Nicola, que está na pasta `RepoNicola/` sem alterações. Todo o código do robô está em **um único arquivo**, `navegador.py`.

## Sumário

1. [Ambiente e instalação](#ambiente-e-instalação)
2. [Como executar](#como-executar)
3. [O que o programa faz](#o-que-o-programa-faz)
4. [Como foi feito](#como-foi-feito)
5. [Estrutura do código](#estrutura-do-código)
6. [Problemas encontrados e soluções](#problemas-encontrados-e-soluções)
7. [Resultados](#resultados)
8. [Glossário](#glossário)
9. [Estrutura do repositório](#estrutura-do-repositório)

## Ambiente e instalação

- Ubuntu 26.04
- ROS 2 Lyrical Luth (pacotes `ros-lyrical-ros-base` e `ros-dev-tools`, instalados pelo apt seguindo a documentação oficial do ROS 2)
- Python 3.14 e `python3-pygame` (`sudo apt install python3-pygame`)

Compilar o simulador (uma vez):

```bash
cd RepoNicola
colcon build
source install/setup.bash
```

Para não precisar repetir o `source` em cada terminal, a linha `source <caminho do repositório>/RepoNicola/install/setup.bash` foi adicionada ao final do `~/.bashrc`.

## Como executar

Use dois terminais.

```bash
# Terminal 1: abrir o jogo (janela "Culling Games")
cd RepoNicola
source install/setup.bash
ros2 run cg maze -- --map 1
```

```bash
# Terminal 2: na raiz do repositório, rodar o robô
source RepoNicola/install/setup.bash
python3 navegador.py          # resolve o labirinto que está aberto no jogo
python3 navegador.py 3        # carrega e resolve o labirinto 3
python3 navegador.py todos    # resolve os 11 labirintos e imprime uma tabela
```

**O que aparece na janela do jogo:** primeiro o robô percorre o labirinto descobrindo o mapa (exploração); depois volta ao início e segue o menor caminho até o alvo, passo a passo; ao chegar, o jogo mostra a tela de vitória.

**No terminal** aparecem o desenho do mapa que o robô construiu (`S` início, `T` alvo, `*` caminho final, `#` parede, `.` livre, `?` não visto), o número de passos e a conferência com o labirinto real. O mesmo texto é salvo em `relatorio.txt`.

**Velocidade do caminho final:** a constante `PAUSA_VISUAL` no topo do `navegador.py` (0,08 s por passo) deixa a janela acompanhar o robô. Aumente (por exemplo, 0,25) para uma demonstração mais lenta.

O modo `todos` leva mais de 15 minutos, porque o jogo demora alguns segundos para recarregar cada labirinto.

## O que o programa faz

Em ordem, para cada labirinto:

1. **Reinicia** o jogo (serviço `/reset`) e espera o labirinto novo estabilizar.
2. **Sente** as 8 células ao redor (tópico de sensores) e anota no mapa.
3. **Explora**: vai até a célula livre mais próxima que ainda tem vizinhas desconhecidas, sente de novo, e repete até não sobrar nenhuma.
4. **Reinicia** de novo, mantendo o mapa na memória.
5. **Navega** pelo menor caminho até o alvo `t`, usando só o mapa que construiu.
6. **Confere** o resultado com o labirinto real (`/get_map`), apenas para avaliação.

## Como foi feito

### Comunicação com o jogo (ROS 2)

- **Tópico** `/culling_games/robot_sensors`: o robô recebe 100 vezes por segundo o que há nas 8 células vizinhas (`b` parede, `f` livre, `t` alvo).
- **Serviço** `/move_command`: pede um passo (`up`, `down`, `left`, `right`). Responde se andou ou se bateu na parede.
- **Serviço** `/reset`: reinicia o robô (e permite trocar de labirinto).
- **Serviço** `/get_map`: devolve o labirinto real. É usado **só para avaliar** se o mapa do robô está certo (o "gabarito"). Nunca é usado para decidir os movimentos.

### Pose e odometria

O robô não sabe onde está no mundo. Ele cria seu próprio referencial, com origem no ponto de partida: começa em `(0, 0)` e, a cada passo que o jogo confirma, soma o deslocamento (odometria). Se o passo bate na parede, a pose não muda. A posição do jogo difere da do robô por um deslocamento constante (só translação, pois o robô não gira). O programa confere isso a cada passo e para com erro se a diferença mudar.

### Mapa de ocupação

Um dicionário `(linha, coluna) -> parede | livre | alvo`. Cada leitura de sensor anota as 8 vizinhas da pose atual. O que não está no dicionário é desconhecido. O robô espera 4 leituras novas antes de usar uma e descarta qualquer leitura que contradiga o que já sabe (o labirinto não muda).

### Exploração por fronteiras

Depois de cada passo o robô escolhe a **célula livre conhecida mais próxima que ainda tem vizinhos desconhecidos** (a *fronteira*), vai até ela e olha de lá. Quando não sobra nenhuma, o mapa está completo. O robô nunca pisa no alvo durante a exploração, porque isso encerraria o jogo.

### Busca em grafo (BFS)

O labirinto é um grafo: cada célula livre é um vértice, ligado às 4 vizinhas, com custo 1 por passo. A busca em largura (BFS) varre o grafo em "ondas" a partir do robô. Por isso o primeiro objetivo que ela encontra é o mais próximo, e o caminho até ele é o menor possível. A mesma função serve para achar a próxima fronteira (na exploração) e o alvo (na navegação final).

**Por que BFS e não Dijkstra ou A\*:** Dijkstra serve quando os passos têm custos diferentes. Com todos os custos iguais a 1, ele se reduz à BFS, com mais código e sem ganho. O A\* acelera a busca com uma heurística (como a distância de Manhattan), mas nesse tamanho de labirinto a BFS já é rápida e mais simples de explicar.

## Estrutura do código

Tudo em `navegador.py`:

| Parte | Função |
|---|---|
| `buscar_caminho` | BFS: devolve a lista de direções até o objetivo mais próximo |
| `contar_celulas_erradas`, `tamanho_do_menor_caminho_real` | avaliação contra o labirinto real |
| `Navegador` (nó ROS 2) | sensores, serviços, pose, mapa, exploração e navegação |
| `Navegador.reiniciar` | reset do jogo com espera de estabilização |
| `Navegador.sentir` | lê os sensores e anota as 8 vizinhas |
| `Navegador.mover` e `conferir_odometria` | dá um passo e confere a odometria |
| `Navegador.explorar` | exploração por fronteiras |
| `Navegador.navegar_ate_alvo` | segue o menor caminho |
| `resolver_labirinto` | roda as etapas e tenta de novo (até 3 vezes) se algo falhar |
| `main` | lê o argumento (`3`, `todos`...), roda e salva `relatorio.txt` |

## Problemas encontrados e soluções

- **Reset assíncrono:** o jogo responde ao `/reset` na hora, mas só recarrega o labirinto alguns segundos depois, e até lá o robô antigo ainda obedece. *Solução:* esperar o `/get_map` mudar e depois ficar igual por 1,5 s.
- **Leituras velhas de sensor:** mensagens de antes do passo podem chegar depois dele. *Solução:* fila de tamanho 1, esperar 4 leituras novas e descartar as que contradizem o mapa.
- **Odometria desalinhada:** se o jogo reinicia no meio da exploração, a odometria continua e o mapa fica corrompido. *Solução:* conferir a cada passo se a diferença para a posição real é constante.
- **Serviço ainda não disponível:** a primeira chamada ficava pendurada. *Solução:* `wait_for_service` antes de começar.
- **Bug do jogo no ROS 2 Lyrical:** o `Game.py` usa `get_logger().warn`, que não existe nessa versão. Qualquer movimento enviado depois da vitória derruba a thread do jogo, que para de responder. *Solução:* o `navegador.py` não envia movimentos quando o jogo já foi ganho (o alvo `t` some do mapa, porque o robô está em cima dele). O código do professor não foi alterado.

## Resultados

Saída de `python3 navegador.py todos` (também em `relatorio.txt`):

```
labirinto    explor.  caminho  ótimo  erros  chegou
1.csv            635      102    102      0  sim
2.csv            705      102    102      0  sim
3.csv            677      192    192      0  sim
4.csv            697      114    114      0  sim
5.csv            619      204    204      0  sim
6.csv            607      110    110      0  sim
7.csv            707       96     96      0  sim
8.csv            653      164    164      0  sim
9.csv            709      190    190      0  sim
10.csv           643       48     48      0  sim
test.csv         702       25     25      0  sim
```

- **explor.**: passos dados para mapear o labirinto.
- **caminho**: passos do caminho final.
- **ótimo**: menor caminho possível, calculado com o labirinto real só para conferir.
- **erros**: células do mapa do robô diferentes do labirinto real (0 = mapa perfeito).
- Nos 11 labirintos o robô mapeou tudo sem erro, o caminho foi o menor possível e ele chegou ao alvo.
- No `test.csv` (campo aberto, sem paredes internas) existem vários caminhos de mesmo tamanho. O robô escolhe um deles, e 25 passos (12 + 13) é o mínimo possível sem andar na diagonal.

## Glossário

**Navegação**

- **Pose:** onde o robô está. Aqui é só `(linha, coluna)`; não há ângulo, pois o robô não gira.
- **Referencial (frame):** o sistema de coordenadas em que uma posição é medida. Temos o **do mundo** (índices da grade do jogo) e o **da odometria** (origem no ponto de partida).
- **Odometria (dead reckoning):** estimar a pose somando os movimentos que deram certo. Em robô real acumula erro, por isso se compara com uma medida absoluta.
- **Mapa de ocupação:** grade em que cada célula é parede, livre ou desconhecida.
- **Fronteira:** célula livre conhecida ao lado de uma região ainda desconhecida. Explorar é ir de fronteira em fronteira.
- **Gabarito (ground truth):** o mapa verdadeiro (`/get_map`). Só serve para medir o erro, nunca para planejar.

**Grafos**

- **Grafo:** vértices (células livres) ligados por arestas (vizinhos de cima, baixo, esquerda, direita), todas com custo 1.
- **BFS (busca em largura):** visita em ondas a partir da origem. Em grafo sem pesos, a primeira vez que chega a um vértice é pelo caminho mais curto.
- **Distância de Manhattan:** |Δlinha| + |Δcoluna|, o menor caminho num campo aberto sem diagonais.

**ROS 2**

- **Nó (node):** programa que participa da rede ROS. O `Navegador` é um nó.
- **Tópico (topic):** canal de mensagens em fluxo contínuo (publicador → assinantes).
- **Serviço (service):** pedido → resposta, como chamar uma função remota.
- **QoS (qualidade de serviço):** regras de entrega do tópico. O de sensores é *best effort*: mensagens podem se perder, mas chegam rápido.
- **Future:** objeto que representa uma resposta que ainda vai chegar (`call_async` devolve um).
- **spin / spin_once:** deixa o ROS processar mensagens e respostas pendentes.
- **wait_for_service:** espera o serviço existir na rede antes da primeira chamada.
- **Thread:** linha de execução paralela dentro do mesmo programa.
- **colcon:** ferramenta que compila o workspace ROS.

## Estrutura do repositório

- `navegador.py`: o programa do robô (arquivo único)
- `README.md`: esta documentação
- `relatorio.txt`: resultado da última execução
- `RepoNicola/`: simulador `culling_games` (código do professor, sem alterações)