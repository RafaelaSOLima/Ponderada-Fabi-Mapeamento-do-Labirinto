import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from cg_interfaces.msg import RobotSensors
from cg_interfaces.srv import GetMap, MoveCmd, Reset

DIRECOES = {'up': (-1, 0), 'down': (1, 0), 'left': (0, -1), 'right': (0, 1)}

VIZINHOS = {
    'up': (-1, 0), 'down': (1, 0), 'left': (0, -1), 'right': (0, 1),
    'up_left': (-1, -1), 'up_right': (-1, 1),
    'down_left': (1, -1), 'down_right': (1, 1),
}

TODOS_OS_MAPAS = [f'{i}.csv' for i in range(1, 11)] + ['test.csv']


def buscar_caminho(inicio, e_objetivo, pode_passar):
    """BFS: devolve a lista de direções até o objetivo MAIS PRÓXIMO.

    e_objetivo(celula)  -> True se a célula é o que procuramos
    pode_passar(celula) -> True se o robô pode andar por ela
    Devolve None se não existe nenhum objetivo alcançável.
    """
    fila = deque([inicio])
    veio_de = {inicio: None}   # célula -> (célula anterior, direção usada)
    while fila:
        atual = fila.popleft()
        if atual != inicio and e_objetivo(atual):
            direcoes = []
            while veio_de[atual] is not None:
                anterior, direcao = veio_de[atual]
                direcoes.append(direcao)
                atual = anterior
            direcoes.reverse()
            return direcoes
        for direcao, (dl, dc) in DIRECOES.items():
            vizinha = (atual[0] + dl, atual[1] + dc)
            if vizinha not in veio_de and pode_passar(vizinha):
                veio_de[vizinha] = (atual, direcao)
                fila.append(vizinha)
    return None


def contar_celulas_erradas(mapa, deslocamento, real):
    """Compara o mapa do robô com o labirinto real (só parede x não-parede).

    deslocamento = posição do mundo que corresponde à origem (0, 0) do robô.
    Célula que o robô nunca viu conta como parede.
    """
    erradas = 0
    for l, linha in enumerate(real):
        for c, valor in enumerate(linha):
            nossa = mapa.get((l - deslocamento[0], c - deslocamento[1]))
            parede_real = valor == 'b'
            parede_nossa = nossa is None or nossa == 'b'
            if parede_real != parede_nossa:
                erradas += 1
    return erradas


def tamanho_do_menor_caminho_real(real, inicio, alvo):
    """Menor caminho possível no labirinto real (para comparar)."""
    def livre(c):
        return (0 <= c[0] < len(real) and 0 <= c[1] < len(real[0])
                and real[c[0]][c[1]] != 'b')
    caminho = buscar_caminho(inicio, lambda c: c == alvo, livre)
    return None if caminho is None else len(caminho)


class Navegador(Node):
    def __init__(self):
        super().__init__('navegador')
        self.sensores = None
        self.contador = 0       # quantas leituras de sensor já chegaram
        self.pose = (0, 0)      # odometria: o robô sempre começa em (0, 0)
        self.mapa = {}          # (linha, coluna) -> 'b' | 'f' | 't'
        self.visitadas = set()  # células onde o robô já esteve
        self.trajeto = []       # células do caminho final até o alvo
        self.deslocamento = None  # posição no jogo = odometria + deslocamento

        # Fila de tamanho 1: assim, no máximo UMA leitura velha (de antes do
        # passo) pode estar esperando. Com a fila padrão (5) o robô podia
        # usar leituras da posição anterior e mapear errado.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            RobotSensors, '/culling_games/robot_sensors',
            self.ao_receber_sensores, qos)
        self.cliente_move = self.create_client(MoveCmd, '/move_command')
        self.cliente_reset = self.create_client(Reset, '/reset')
        self.cliente_mapa = self.create_client(GetMap, '/get_map')

    def ao_receber_sensores(self, msg):
        self.sensores = msg
        self.contador += 1

    def esperar_conexao(self):
        """Espera o jogo aparecer na rede antes de usar os serviços."""
        for cliente in (self.cliente_move, self.cliente_reset,
                        self.cliente_mapa):
            while not cliente.wait_for_service(timeout_sec=1.0):
                print('esperando o jogo... (ros2 run cg maze está aberto?)')
        while self.sensores is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def esperar(self, futuro, limite=20.0):
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=limite)
        if not futuro.done():
            raise RuntimeError(f'o jogo não respondeu em {limite:.0f} s')
        return futuro.result()

    def afastar_do_inicio(self):
        """Dá um passo qualquer, só para o estado do jogo ficar diferente do
        estado inicial. Assim dá para notar QUANDO o reset realmente vale."""
        for direcao in DIRECOES:
            if self.mover(direcao).success:
                return

    def reiniciar(self, nome_mapa='', manter_mapa=False):
        """Volta o robô ao início.

        nome_mapa vazio -> recarrega o labirinto atual.
        nome_mapa 'X.csv' -> carrega esse labirinto.
        manter_mapa=True -> o robô não esquece o que já mapeou.
        """
        print('  reiniciando o jogo...', flush=True)
        # Se o jogo já foi ganho (o alvo 't' sumiu do mapa, porque o robô
        # está em cima dele), NÃO mandamos nenhum movimento: nesse estado o
        # jogo do Nicola quebra (bug do get_logger().warn no ROS 2 Lyrical)
        # e para de responder de vez.
        if any('t' in linha for linha in self.pegar_mapa_real()):
            self.afastar_do_inicio()
        antes = self.pegar_mapa_real()
        pedido = Reset.Request()
        pedido.is_random = False
        pedido.map_name = nome_mapa
        resposta = self.esperar(self.cliente_reset.call_async(pedido))
        if not resposta.success:
            raise RuntimeError(f'o jogo não achou o labirinto "{nome_mapa}"')
        # O jogo responde na hora, mas só recarrega o labirinto alguns
        # segundos depois (e até lá o robô antigo ainda obedece!). Esperamos o
        # labirinto realmente mudar.
        limite = time.monotonic() + 15.0
        while self.pegar_mapa_real() == antes:
            if time.monotonic() > limite:
                raise RuntimeError('o jogo demorou demais para reiniciar')
            time.sleep(0.05)
        # O labirinto novo pode aparecer em etapas. Esperamos ele ficar
        # IGUAL por 1,5 s seguidos antes de confiar nele.
        estavel_desde = time.monotonic()
        atual = self.pegar_mapa_real()
        while time.monotonic() - estavel_desde < 1.5:
            time.sleep(0.1)
            novo = self.pegar_mapa_real()
            if novo != atual:
                atual = novo
                estavel_desde = time.monotonic()
            if time.monotonic() > limite + 30.0:
                raise RuntimeError('o labirinto não parou de mudar')
        print('  jogo reiniciado.', flush=True)
        self.pose = (0, 0)
        self.deslocamento = None
        if not manter_mapa:
            self.mapa = {}
            self.visitadas = set()

    def conflitos_da_leitura(self):
        """Vizinhas que a leitura atual contradiz em relação ao mapa."""
        conflitos = []
        for campo, (dl, dc) in VIZINHOS.items():
            celula = (self.pose[0] + dl, self.pose[1] + dc)
            novo = getattr(self.sensores, campo)
            antigo = self.mapa.get(celula)
            if antigo is not None and antigo != novo:
                conflitos.append((campo, celula, antigo, novo))
        return conflitos

    def sentir(self):
        """Espera leituras NOVAS e anota os 8 vizinhos no mapa.

        O labirinto não muda, então uma leitura boa tem que concordar com o
        que já sabemos. Se discordar, quase certamente é uma leitura velha
        (de antes do passo): descartamos e esperamos outra.
        """
        for _ in range(5):
            alvo = self.contador + 4
            limite = time.monotonic() + 10.0
            while self.contador < alvo:
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.monotonic() > limite:
                    raise RuntimeError('o jogo parou de enviar sensores')
            conflitos = self.conflitos_da_leitura()
            if not conflitos:
                break
        else:
            raise RuntimeError(
                f'os sensores insistem em contradizer o mapa na pose '
                f'{self.pose}. (campo, célula, o que eu sabia, o que li): '
                f'{conflitos[:3]}')
        self.mapa[self.pose] = 'f'
        self.visitadas.add(self.pose)
        for campo, (dl, dc) in VIZINHOS.items():
            celula = (self.pose[0] + dl, self.pose[1] + dc)
            self.mapa[celula] = getattr(self.sensores, campo)

    def mover(self, direcao):
        pedido = MoveCmd.Request()
        pedido.direction = direcao
        resposta = self.esperar(self.cliente_move.call_async(pedido))
        if resposta.success:    # só soma se o robô andou de fato
            dl, dc = DIRECOES[direcao]
            self.pose = (self.pose[0] + dl, self.pose[1] + dc)
        self.conferir_odometria(resposta)
        return resposta

    def conferir_odometria(self, resposta):
        """Compara a odometria com a posição real que o jogo informa.

        A diferença entre as duas (o deslocamento) tem que ser SEMPRE a mesma.
        Se mudar, o robô se perdeu (ex.: o jogo foi reiniciado no meio).
        """
        mundo = (int(resposta.robot_pos[0]), int(resposta.robot_pos[1]))
        diferenca = (mundo[0] - self.pose[0], mundo[1] - self.pose[1])
        if self.deslocamento is None:
            self.deslocamento = diferenca
        elif diferenca != self.deslocamento:
            raise RuntimeError(
                'a odometria discorda da posição real do robô no jogo '
                f'(odometria {self.pose}, jogo {mundo}). O jogo foi '
                'reiniciado no meio da execução?')

    def precisa_visitar(self, celula):
        """Livre, ainda não visitada e com algum vizinho desconhecido."""
        if self.mapa.get(celula) != 'f' or celula in self.visitadas:
            return False
        return any((celula[0] + dl, celula[1] + dc) not in self.mapa
                   for dl, dc in VIZINHOS.values())

    def explorar(self):
        """Explora até não sobrar nenhuma fronteira. Devolve nº de passos."""
        passos = 0
        while True:
            caminho = buscar_caminho(
                self.pose,
                e_objetivo=self.precisa_visitar,
                pode_passar=lambda c: self.mapa.get(c) == 'f')
            if caminho is None:
                return passos
            for direcao in caminho:
                resposta = self.mover(direcao)
                passos += 1
                self.sentir()
                if not resposta.success:
                    break       # algo inesperado: replaneja

    def navegar_ate_alvo(self):
        """Segue o menor caminho (pelo mapa) até a célula do alvo."""
        caminho = buscar_caminho(
            self.pose,
            e_objetivo=lambda c: self.mapa.get(c) == 't',
            pode_passar=lambda c: self.mapa.get(c) in ('f', 't'))
        if caminho is None:
            raise RuntimeError('não existe caminho conhecido até o alvo')
        self.trajeto = [self.pose]
        for direcao in caminho:
            resposta = self.mover(direcao)
            self.trajeto.append(self.pose)
        return len(caminho), resposta

    def pegar_mapa_real(self):
        """Pede ao jogo o labirinto real. SÓ para avaliar o nosso mapa."""
        resposta = self.esperar(self.cliente_mapa.call_async(GetMap.Request()))
        linhas, colunas = (int(v) for v in resposta.occupancy_grid_shape)
        plano = list(resposta.occupancy_grid_flattened)
        return [plano[i * colunas:(i + 1) * colunas] for i in range(linhas)]

    def desenhar(self):
        """Desenho do mapa. S=início, T=alvo, *=caminho, #=parede, ?=não visto."""
        if not self.mapa:
            return '(mapa vazio)'
        linhas = [c[0] for c in self.mapa]
        colunas = [c[1] for c in self.mapa]
        caminho = set(self.trajeto)
        texto = []
        for l in range(min(linhas), max(linhas) + 1):
            linha = ''
            for c in range(min(colunas), max(colunas) + 1):
                celula = (l, c)
                valor = self.mapa.get(celula)
                if celula == (0, 0):
                    linha += 'S'
                elif valor == 't':
                    linha += 'T'
                elif celula in caminho:
                    linha += '*'
                elif valor == 'b':
                    linha += '#'
                elif valor == 'f':
                    linha += '.'
                else:
                    linha += '?'
            texto.append(linha)
        return '\n'.join(texto)


def resolver_labirinto(robo, nome_mapa='', tentativas=3):
    """Resolve o labirinto; se algo der errado no meio, tenta de novo."""
    for tentativa in range(1, tentativas + 1):
        try:
            return resolver_uma_vez(robo, nome_mapa)
        except RuntimeError as erro:
            if tentativa == tentativas:
                raise
            print(f'  tentativa {tentativa} falhou ({erro}); '
                  f'esperando o jogo assentar e tentando de novo...')
            time.sleep(3.0 * tentativa)


def resolver_uma_vez(robo, nome_mapa=''):
    """Mapeia o labirinto, segue o menor caminho e devolve os resultados."""
    robo.reiniciar(nome_mapa)
    robo.sentir()
    passos_exploracao = robo.explorar()
    livres = sum(1 for v in robo.mapa.values() if v != 'b')

    robo.reiniciar(manter_mapa=True)
    tamanho, resposta = robo.navegar_ate_alvo()

    mundo_final = tuple(int(v) for v in resposta.robot_pos)
    alvo_mundo = tuple(int(v) for v in resposta.target_pos)
    deslocamento = (mundo_final[0] - robo.pose[0],
                    mundo_final[1] - robo.pose[1])
    real = robo.pegar_mapa_real()
    return {
        'mapa': nome_mapa or 'atual',
        'chegou': mundo_final == alvo_mundo,
        'passos_exploracao': passos_exploracao,
        'celulas_livres': livres,
        'caminho': tamanho,
        'otimo': tamanho_do_menor_caminho_real(real, deslocamento,
                                               mundo_final),
        'celulas_erradas': contar_celulas_erradas(robo.mapa, deslocamento,
                                                  real),
        'desenho': robo.desenhar(),
    }


def texto_do_resultado(r):
    return '\n'.join([
        f'=== Labirinto {r["mapa"]} ===',
        r['desenho'],
        '',
        f'Exploração: {r["passos_exploracao"]} passos',
        f'Células livres descobertas: {r["celulas_livres"]}',
        f'Caminho até o alvo: {r["caminho"]} passos '
        f'(menor possível: {r["otimo"]})',
        f'Células do mapa diferentes do labirinto real: '
        f'{r["celulas_erradas"]}',
        f'Chegou ao alvo: {"sim" if r["chegou"] else "NÃO"}',
    ])


def texto_da_tabela(resultados):
    linhas = [f'{"labirinto":11}{"explor.":>9}{"caminho":>9}{"ótimo":>7}'
              f'{"erros":>7}  chegou']
    for r in resultados:
        linhas.append(f'{r["mapa"]:11}{r["passos_exploracao"]:>9}'
                      f'{r["caminho"]:>9}{r["otimo"]:>7}'
                      f'{r["celulas_erradas"]:>7}  '
                      f'{"sim" if r["chegou"] else "NÃO"}')
    return '\n'.join(linhas)


def main():
    argumento = sys.argv[1] if len(sys.argv) > 1 else ''
    if argumento == 'todos':
        nomes = TODOS_OS_MAPAS
    elif argumento:
        nomes = [argumento if argumento.endswith('.csv')
                 else argumento + '.csv']
    else:
        nomes = ['']            # o labirinto que já está aberto no jogo

    rclpy.init()
    robo = Navegador()
    robo.esperar_conexao()

    resultados = []
    textos = []
    for nome in nomes:
        print(f'Resolvendo o labirinto {nome or "atual"}...')
        try:
            resultado = resolver_labirinto(robo, nome)
        except RuntimeError as erro:
            aviso = (f'ERRO no labirinto {nome or "atual"}: {erro}\n'
                     f'Posição do robô (odometria): {robo.pose}\n'
                     f'Mapa que o robô tinha neste momento:\n'
                     f'{robo.desenhar()}')
            print(aviso)
            textos.append(aviso)
            break
        resultados.append(resultado)
        textos.append(texto_do_resultado(resultado))
        print(textos[-1] + '\n')

    if len(resultados) > 1:
        textos.append(texto_da_tabela(resultados))
        print(textos[-1])

    with open('relatorio.txt', 'w', encoding='utf-8') as arquivo:
        arquivo.write('\n\n'.join(textos) + '\n')
    print('\nRelatório salvo em relatorio.txt')

    robo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()