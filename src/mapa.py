import random
from matplotlib import path as mpath
from geometry_msgs.msg import Point

class Mapa:
    def __init__(self):
        """
        Inicializa o mapa com os pontos predefinidos que formam o polígono.
        """
        self.pontos_mapa = [
            [-21, 3], [-21, -3], [-11, -3], [-11, -8], [11, -8], [11, -3],
            [21, -3], [21, 3], [11, 3], [11, 8], [-11, 8], [-11, 3]
        ]

    def verifica_ponto_dentro(self, ponto):
        """
        Verifica se um ponto (x, y) está dentro do polígono definido pelos limites do mapa.
        """
        path = mpath.Path(self.pontos_mapa)
        return path.contains_point(ponto)
    
    def verifica_ponto_dentro_filtro(self, ponto):
        """
        Verifica se um ponto (x, y) está dentro do polígono definido pelos limites do mapa.
        """
        path = mpath.Path(self.pontos_mapa)
        return path.contains_point((ponto.x, ponto.y))

    def gerar_pontos_aleatorios_dentro(self, num_pontos):
        """
        Gera pontos aleatórios que estejam dentro do mapa.
        """
        pontos_validos = []
        
        # Bounding box para limitar a geração de pontos, com base nos limites máximos e mínimos do mapa
        min_x = min(p[0] for p in self.pontos_mapa)
        max_x = max(p[0] for p in self.pontos_mapa)
        min_y = min(p[1] for p in self.pontos_mapa)
        max_y = max(p[1] for p in self.pontos_mapa)

        while len(pontos_validos) < num_pontos:
            # Gerar ponto aleatório dentro da bounding box
            ponto = Point()
            ponto.x = random.uniform(min_x, max_x)
            ponto.y = random.uniform(min_y, max_y)
            ponto.z = 0.0  # Manter z como 0 para 2D

            # Verificar se o ponto está dentro dos limites do polígono
            if self.verifica_ponto_dentro_filtro(ponto):
                pontos_validos.append(ponto)

        return pontos_validos

    def desenhar_mapa(self, ax):
        """
        Desenha o mapa representado pelas linhas conectando os pontos no gráfico fornecido.
        """
        x_vals = [p[0] for p in self.pontos_mapa] + [self.pontos_mapa[0][0]]  # Fechando o polígono
        y_vals = [p[1] for p in self.pontos_mapa] + [self.pontos_mapa[0][1]]
        ax.plot(x_vals, y_vals, 'k-', label='Mapa')

    def __repr__(self):
        return f'MapaComLimites(pontos={self.pontos_mapa})'
