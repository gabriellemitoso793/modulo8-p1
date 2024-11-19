import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, GetMap
import numpy as np
import time

# valida o mapa
def validar_mapa(mapa):
    if not all(isinstance(row, list) for row in mapa):
        raise ValueError("O mapa deve ser uma lista de listas.")
    if not all(len(row) == len(mapa[0]) for row in mapa):
        raise ValueError("Todas as linhas do mapa devem ter o mesmo comprimento.")
    return True


class NavegacaoPorMapa(Node):
    def __init__(self):
        super().__init__('navegacao_por_mapa')

        # Cliente para o serviço de movimentação
        self.cliente_movimento = self.create_client(MoveCmd, '/move_command')
        while not self.cliente_movimento.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando pelo serviço de movimentação...')
        
        # Cliente para o serviço de mapa
        self.cliente_mapa = self.create_client(GetMap, '/get_map')
        while not self.cliente_mapa.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando pelo serviço de mapa...')
        
        self.posicao_alvo = [18, 18]  # Posição padrão do alvo (será ajustada com o mapa real)
        self.dados_mapa = None  # Armazena o mapa processado
        self.caminho = []  # Caminho planejado do robô

    def obter_mapa(self):
        """Obtém o mapa e identifica as posições de início e objetivo."""
        requisicao = GetMap.Request()
        futuro = self.cliente_mapa.call_async(requisicao)
        rclpy.spin_until_future_complete(self, futuro)
        resposta = futuro.result()
        if resposta:
            self.get_logger().info("Mapa recebido com sucesso.")
            try:
                # Converte o mapa para uma matriz bidimensional
                formato = tuple(resposta.occupancy_grid_shape)
                mapa_linearizado = resposta.occupancy_grid_flattened
                
                # Converte o mapa em uma matriz de strings
                self.dados_mapa = np.array(mapa_linearizado, dtype=str).reshape(formato)
                
                # Identifica as posições do robô (r) e do objetivo (t)
                inicio = np.argwhere(self.dados_mapa == 'r')
                objetivo = np.argwhere(self.dados_mapa == 't')
                
                if len(inicio) > 0 and len(objetivo) > 0:
                    self.posicao_inicial = tuple(inicio[0])  # Posição inicial do robô
                    self.posicao_alvo = tuple(objetivo[0])   # Posição do objetivo
                    self.get_logger().info(f"Posição inicial: {self.posicao_inicial}, Alvo: {self.posicao_alvo}")
                else:
                    self.get_logger().error("Não foi possível encontrar as posições de início ou objetivo.")
                    self.posicao_inicial = None
                    self.posicao_alvo = None
                
                # Converte o mapa para valores numéricos (0: livre, 1: obstáculo)
                self.dados_mapa = np.where(self.dados_mapa == 'f', 0, 1)
                self.dados_mapa[self.posicao_inicial] = 0  # Garante que o início seja acessível
                self.dados_mapa[self.posicao_alvo] = 0     # Garante que o alvo seja acessível

            except Exception as e:
                self.get_logger().error(f"Erro ao processar o mapa: {e}")
                self.dados_mapa = None

            return self.dados_mapa
        else:
            self.get_logger().error('Falha ao obter o mapa.')
            return None

    def a_star(self, inicio, objetivo):
        """Implementação do algoritmo A*."""
        def distancia_manhattan(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        # Conjuntos abertos e fechados
        conjunto_aberto = {inicio}
        origem = {}
        g_score = {inicio: 0}
        f_score = {inicio: distancia_manhattan(inicio, objetivo)}

        while conjunto_aberto:
            # Seleciona o nó com menor custo estimado
            atual = min(conjunto_aberto, key=lambda x: f_score.get(x, float('inf')))
            if atual == objetivo:
                # Reconstrói o caminho
                caminho = []
                while atual in origem:
                    caminho.append(atual)
                    atual = origem[atual]
                caminho.append(inicio)
                caminho.reverse()
                return caminho

            conjunto_aberto.remove(atual)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                vizinho = (atual[0] + dx, atual[1] + dy)
                if (0 <= vizinho[0] < self.dados_mapa.shape[0] and
                    0 <= vizinho[1] < self.dados_mapa.shape[1] and
                    self.dados_mapa[vizinho] == 0):  # Célula livre
                    custo_tentativo = g_score[atual] + 1
                    if custo_tentativo < g_score.get(vizinho, float('inf')):
                        origem[vizinho] = atual
                        g_score[vizinho] = custo_tentativo
                        f_score[vizinho] = custo_tentativo + distancia_manhattan(vizinho, objetivo)
                        conjunto_aberto.add(vizinho)

        return []  # Nenhum caminho encontrado

    def planejar_rota(self):
        """Planeja a rota para o robô usando A*."""
        self.get_logger().info("Planejando a rota...")
        if self.dados_mapa is not None and self.posicao_inicial and self.posicao_alvo:
            self.caminho = self.a_star(self.posicao_inicial, self.posicao_alvo)
            if self.caminho:
                self.get_logger().info(f"Rota planejada: {self.caminho}")
            else:
                self.get_logger().error("Não foi possível planejar uma rota.")
        else:
            self.get_logger().error("Dados insuficientes para planejar a rota.")

    def navegar_rota(self):
        """Navega pelo caminho planejado."""
        posicao_atual = self.posicao_inicial
        for posicao_destino in self.caminho[1:]:
            if posicao_atual == self.posicao_alvo:
                self.get_logger().info("O robô alcançou o alvo!")
                break

            direcao = self.determinar_direcao(posicao_atual, posicao_destino)
            if not direcao:
                self.get_logger().error(f"Direção inválida para {posicao_destino}.")
                break

            resposta = self.mover_robo(direcao)
            if resposta and not resposta.success:
                self.get_logger().error("Falha ao mover o robô.")
                break

            posicao_atual = posicao_destino
            self.get_logger().info(f"Nova posição: {posicao_atual}")
            time.sleep(0.5)  # Delay para simular o movimento

    def determinar_direcao(self, posicao_atual, posicao_destino):
        """Determina a direção para o próximo passo."""
        dx = posicao_destino[0] - posicao_atual[0]
        dy = posicao_destino[1] - posicao_atual[1]

        if dx == 0 and dy == 1:
            return 'right'
        elif dx == 0 and dy == -1:
            return 'left'
        elif dx == 1 and dy == 0:
            return 'down'
        elif dx == -1 and dy == 0:
            return 'up'
        else:
            self.get_logger().error(f"Erro ao calcular direção: dx={dx}, dy={dy}")
            return None

    def mover_robo(self, direcao):
        """Move o robô em uma direção específica."""
        requisicao = MoveCmd.Request()
        requisicao.direction = direcao
        futuro = self.cliente_movimento.call_async(requisicao)
        rclpy.spin_until_future_complete(self, futuro)
        resposta = futuro.result()
        if resposta:
            self.get_logger().info(f"Movimento: {direcao} | Sucesso: {resposta.success}")
            return resposta
        else:
            self.get_logger().error("Erro ao mover o robô.")
            return None


def main(args=None):
    rclpy.init(args=args)
    cliente = NavegacaoPorMapa()
    cliente.obter_mapa()
    cliente.planejar_rota()
    cliente.navegar_rota()
    cliente.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
