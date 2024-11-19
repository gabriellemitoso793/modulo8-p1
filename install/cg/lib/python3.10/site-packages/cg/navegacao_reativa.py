import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
import time

# DFS e backtracking 

class ReactiveNavigation(Node):
    def __init__(self):
        super().__init__('reactive_navigation')
        
        # cria o cliente para o serviço move_command enviar os comandos de movimento
        self.move_client = self.create_client(MoveCmd, '/move_command')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /move_command...')
        
        # guarda as posições que o robô já foi para evitar loop
        self.visited_positions = set()
        
        # pilha que armazena as posições que tomam a decisão para evitar o loop
        self.backtrack_stack = []
    
    def send_move_request(self, direction):
        # envia uma solicitação de movimento na direção
        request = MoveCmd.Request()

        # define como o robô vai se mexer up, down, left ou right
        request.direction = direction

        # programa continua rodando enquanto processa o comando
        future = self.move_client.call_async(request)

        # Aguarda que o serviço processe a solicitação e retorne um resultado.
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        #timer para percorrer mais devagar
        time.sleep(0.1)
        return result
    
    # retorna uma lista de direções que o robo ainda não foi
    def get_unvisited_directions(self, robot_pos, sensors):
        directions = []
        for direction, sensor in sensors.items():
            
            # identifica se é espaço livre ou o target
            if sensor in ['f', 't']:
                
                # calcula a nova posição
                new_pos = list(robot_pos)
                if direction == 'left':
                    new_pos[1] -= 1
                elif direction == 'right':
                    new_pos[1] += 1
                elif direction == 'up':
                    new_pos[0] -= 1
                elif direction == 'down':
                    new_pos[0] += 1
                
                # adiciona a direção se a nova posição ainda não foi visitada
                if tuple(new_pos) not in self.visited_positions:
                    directions.append((direction, tuple(new_pos)))
        return directions
    
    # basicamente vai determinar as melhores direções para o robô ir
    def prioritize_directions_toward_target(self, robot_pos, target_pos, sensors):
        directions = []
        
        # identifica se ta livre ou se é o alvo
        if robot_pos[0] < target_pos[0] and sensors['down'] in ['f', 't']:
            directions.append(('down', (robot_pos[0] + 1, robot_pos[1])))
        if robot_pos[0] > target_pos[0] and sensors['up'] in ['f', 't']:
            directions.append(('up', (robot_pos[0] - 1, robot_pos[1])))
        if robot_pos[1] < target_pos[1] and sensors['right'] in ['f', 't']:
            directions.append(('right', (robot_pos[0], robot_pos[1] + 1)))
        if robot_pos[1] > target_pos[1] and sensors['left'] in ['f', 't']:
            directions.append(('left', (robot_pos[0], robot_pos[1] - 1)))
        return directions
    
    # implementa a navegação reativa
    def reactive_navigation(self):
        while rclpy.ok():
            
            # requisição spara ler sensores
            result = self.send_move_request('')
            robot_pos = tuple(result.robot_pos)
            target_pos = tuple(result.target_pos)
            left, down, up, right = result.left, result.down, result.up, result.right
            sensors = {'left': left, 'down': down, 'up': up, 'right': right}
            
            # quebra o loop se chegar no alvo
            if robot_pos == target_pos:
                self.get_logger().info('Alvo alcançado!')
                break
            # adiciona a posição que o robô tá no meu dicionário pata evitar que ele passe de novo
            self.visited_positions.add(robot_pos)
            # verifica se o robô ta perto do target
            if self.is_near_target(robot_pos, target_pos, distance=3):
                priority_directions = self.prioritize_directions_toward_target(robot_pos, target_pos, sensors)
                if priority_directions:
                    direction, new_pos = priority_directions[0]
                    move_result = self.send_move_request(direction)
                    if move_result.success:
                        self.get_logger().info(f"Movimento bem-sucedido para {direction}. Nova posição: {move_result.robot_pos}")
                        continue
            
            # vê as direções que ainda nao foram e colocam a atual de volta na pilha
            unvisited_directions = self.get_unvisited_directions(robot_pos, sensors)
            if len(unvisited_directions) > 1:
                self.backtrack_stack.append(robot_pos)
            
            # vê se ainda tem posições para percorrer, envia comando para andar e guarda a nova posição
            if unvisited_directions:
                direction, new_pos = unvisited_directions[0]
                move_result = self.send_move_request(direction)
                if move_result.success:
                    self.get_logger().info(f"Movimento bem-sucedido para {direction}. Nova posição: {move_result.robot_pos}")
            
            # verifica se tem posição na pilha
            else:
                
                # se tiver, remove a ultima e leva o robô ate la
                if self.backtrack_stack:
                    self.get_logger().info('Nenhuma direção nova, voltando para último nó com opções...')
                    backtrack_pos = self.backtrack_stack.pop()
                    self.move_to_position(backtrack_pos)
                
                # se não tiver, faz o break
                else:
                    self.get_logger().info('Sem movimentos válidos disponíveis e sem mais nós para voltar. Robô preso.')
                    break
    
    # navega até a posição alvo (ultima da pilha)
    def move_to_position(self, target_pos):
        
        # pega a posição atual do robô e guarda como tupla
        current_result = self.send_move_request('')
        robot_pos = tuple(current_result.robot_pos)
        
        # continua movendo o robô até chegar no target
        while robot_pos != target_pos:
            # compara as coord do robô e do alvo e diz para onde o robô tem que ir
            direction = None
            if robot_pos[0] < target_pos[0]:
                direction = 'down'
            elif robot_pos[0] > target_pos[0]:
                direction = 'up'
            elif robot_pos[1] < target_pos[1]:
                direction = 'right'
            elif robot_pos[1] > target_pos[1]:
                direction = 'left'
            # continua movendo ate encontrar um obstaculo
            move_result = self.send_move_request(direction)
            if move_result.success:
                robot_pos = tuple(move_result.robot_pos)
                self.get_logger().info(f"Backtracking para {direction}. Nova posição: {robot_pos}")
            else:
                self.get_logger().info(f"Falha ao voltar para {direction}. Tentando outra direção.")
                break
    
    # usa o metodo de manhattan para saber se o robo ta perto do alvo ou nao
    def is_near_target(self, robot_pos, target_pos, distance=2):
        return abs(robot_pos[0] - target_pos[0]) + abs(robot_pos[1] - target_pos[1]) <= distance

# inicia o nó executa e fecha o ros
def main():
    rclpy.init()
    navigator = ReactiveNavigation()
    navigator.reactive_navigation()
    rclpy.shutdown()

# main basica para garantir que ta ok!
if __name__ == '__main__':
    main()