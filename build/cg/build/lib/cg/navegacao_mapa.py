import rclpy
from rclpy.node import Node
from cg_interfaces.srv import GetMap, MoveCmd
import heapq


class NavegacaoMapa(Node):
    def _init_(self):
        super()._init_('navegacao_mapa')
        self.get_logger().info("Nó de Navegação com Mapa iniciado")
        self.cliente_map = self.create_client(GetMap, '/get_map')
        self.cliente_move = self.create_client(MoveCmd, '/move_command')

        while not self.cliente_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando pelo serviço /get_map...')
        while not self.cliente_move.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando pelo serviço /move_command...')

    def reconstruir_mapa(self, occupancy_grid_flattened, shape):
        rows, cols = shape
        return [occupancy_grid_flattened[i * cols:(i + 1) * cols] for i in range(rows)]

    def encontrar_rota(self, mapa, inicio, alvo):
        filas = [(0, inicio)]
        custos = {inicio: 0}
        caminhos = {inicio: None}

        while filas:
            _, atual = heapq.heappop(filas)

            if atual == alvo:
                break

            for d, (dr, dc) in zip(
                ['up', 'down', 'left', 'right'], [(-1, 0), (1, 0), (0, -1), (0, 1)]
            ):
                vizinho = (atual[0] + dr, atual[1] + dc)
                if 0 <= vizinho[0] < len(mapa) and 0 <= vizinho[1] < len(mapa[0]):
                    if mapa[vizinho[0]][vizinho[1]] != 'b':
                        novo_custo = custos[atual] + 1
                        if vizinho not in custos or novo_custo < custos[vizinho]:
                            custos[vizinho] = novo_custo
                            heapq.heappush(filas, (novo_custo, vizinho))
                            caminhos[vizinho] = (atual, d)

        caminho = []
        atual = alvo
        while atual != inicio:
            atual, direcao = caminhos[atual]
            caminho.append(direcao)
        return caminho[::-1]

    def navegar(self):
        # Solicita o mapa
        req_map = GetMap.Request()
        future_map = self.cliente_map.call_async(req_map)
        rclpy.spin_until_future_complete(self, future_map)

        if future_map.result():
            mapa = self.reconstruir_mapa(
                future_map.result().occupancy_grid_flattened,
                future_map.result().occupancy_grid_shape,
            )
            pos_inicial = (future_map.result().robot_pos[0], future_map.result().robot_pos[1])
            pos_alvo = (future_map.result().target_pos[0], future_map.result().target_pos[1])

            rota = self.encontrar_rota(mapa, pos_inicial, pos_alvo)

            # Navega até o destino
            for direcao in rota:
                req_move = MoveCmd.Request()
                req_move.direction = direcao
                future_move = self.cliente_move.call_async(req_move)
                rclpy.spin_until_future_complete(self, future_move)

                if not future_move.result().success:
                    self.get_logger().warning(f"Movimento {direcao} falhou.")
                    break
                self.get_logger().info(f"Movendo para {direcao}")
        else:
            self.get_logger().error("Falha ao obter o mapa.")


def main(args=None):
    rclpy.init(args=args)
    nodo_navegacao_mapa = NavegacaoMapa()
    nodo_navegacao_mapa.navegar()
    nodo_navegacao_mapa.destroy_node()
    rclpy.shutdown()