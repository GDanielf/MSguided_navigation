import random
from matplotlib import path as mpath
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
import numpy as np

class Mapa:
    def __init__(self):
        """
        Inicializa o mapa com os pontos predefinidos que formam o polígono e a grade de regiões de risco.
        """
        self.pontos_mapa = [
            [-21, 3], [-21, -3], [-11, -3], [-11, -8], [11, -8], [11, -3],
            [21, -3], [21, 3], [11, 3], [11, 8], [-11, 8], [-11, 3]
        ]

        # Configuração da grade
        self.tamanho_celula = 2  # Tamanho do quadrado em metros (2x2)
        self.x_min = -10
        self.x_max = 10
        self.y_min = -7.5
        self.y_max = 7.5
        self.grade = self._criar_grade_de_risco()    

    def _criar_grade_de_risco(self):
        """
        Cria a grade de regiões do mapa com base no tamanho das células e limites.
        As regiões são classificadas em vermelho, amarelo e verde.
        """
        grade = {}
        contador_cima, contador_baixo = 0, 0
        contador_esq, contador_dir = 0, 0

        for i in np.arange(self.x_min, self.x_max, self.tamanho_celula):
            for j in np.arange(self.y_min, self.y_max, self.tamanho_celula):
                # Determinar a cor e nome do quadrado
                if (i == self.x_min or i + self.tamanho_celula >= self.x_max or
                        j == self.y_min or j + self.tamanho_celula >= self.y_max):
                    if j > 0:
                        cor = f'vermelha_cima_{contador_cima}'
                        contador_cima += 1
                    elif j < 0:
                        cor = f'vermelha_baixo_{contador_baixo}'
                        contador_baixo += 1
                    elif i < 0:
                        cor = f'vermelha_esq_{contador_esq}'
                        contador_esq += 1
                    else:
                        cor = f'vermelha_dir_{contador_dir}'
                        contador_dir += 1
                elif (i <= self.x_min + self.tamanho_celula or i + self.tamanho_celula >= self.x_max - self.tamanho_celula or
                      j <= self.y_min + self.tamanho_celula or j + self.tamanho_celula >= self.y_max - self.tamanho_celula):
                    cor = 'amarelo'  # Próximo aos extremos
                else:
                    cor = 'verde'  # Região central
                grade[(round(i, 2), round(j, 2))] = cor
        return grade

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

    def obter_cor_regiao(self, x, y):
        """
        Determina a cor da região do mapa onde o ponto (x, y) está localizado.
        """
        # Encontrar a célula correspondente ao ponto
        x_celula = round(x // self.tamanho_celula * self.tamanho_celula, 2)
        y_celula = round(y // self.tamanho_celula * self.tamanho_celula, 2)
        return self.grade.get((x_celula, y_celula), 'fora_do_mapa')

    def desenhar_mapa(self, ax):
        """
        Desenha o mapa representado pelas linhas conectando os pontos no gráfico fornecido.
        Também plota a grade colorida para visualização.
        """
        # Desenhar os limites do mapa
        x_vals = [p[0] for p in self.pontos_mapa] + [self.pontos_mapa[0][0]]  # Fechando o polígono
        y_vals = [p[1] for p in self.pontos_mapa] + [self.pontos_mapa[0][1]]
        ax.plot(x_vals, y_vals, 'k-', label='Mapa')

        # Desenhar a grade
        for (x, y), cor in self.grade.items():
            if 'vermelha' in cor:
                ax.add_patch(plt.Rectangle((x, y), self.tamanho_celula, self.tamanho_celula, color='red', alpha=0.3))
            elif cor == 'amarelo':
                ax.add_patch(plt.Rectangle((x, y), self.tamanho_celula, self.tamanho_celula, color='yellow', alpha=0.3))
            elif cor == 'verde':
                ax.add_patch(plt.Rectangle((x, y), self.tamanho_celula, self.tamanho_celula, color='green', alpha=0.3))

    def __repr__(self):
        return f'MapaComLimites(pontos={self.pontos_mapa})'

# Exemplo de uso
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    mapa = Mapa()

    # Teste para verificar a cor de uma posição
    x, y = 1.3, -6.9
    cor = mapa.obter_cor_regiao(x, y)
    print(f"A posição ({x}, {y}) está na região: {cor}")

    # Plotar o mapa
    fig, ax = plt.subplots()
    mapa.desenhar_mapa(ax)
    plt.axis('equal')
    plt.legend()
    plt.show()
