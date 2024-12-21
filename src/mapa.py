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

        self.regioes = {
            # Vermelha (Baixo)
            (-10, -8, -7.5, -5.5): 0,   #"vermelha_baixo_0"
            (-10, -8, -5.5, -3.5): 1,   #"vermelha_baixo_1"
            (-10, -8, -3.5, -1.5): 2,   #"vermelha_baixo_2"
            (-10, -8, -1.5, 0.5): 3,    #"vermelha_baixo_3"
            (-8, -6, -7.5, -5.5): 4,    #"vermelha_baixo_4"
            (-6, -4, -7.5, -5.5): 5,    #"vermelha_baixo_5"
            (-4, -2, -7.5, -5.5): 6,    #"vermelha_baixo_6"
            (-2, 0, -7.5, -5.5): 7,     #"vermelha_baixo_7"
            (0, 2, -7.5, -5.5): 8,      #"vermelha_baixo_8"
            (2, 4, -7.5, -5.5): 9,      #"vermelha_baixo_9"
            (4, 6, -7.5, -5.5): 10,     #"vermelha_baixo_10"
            (6, 8, -7.5, -5.5): 11,     #"vermelha_baixo_11"
            (8, 10, -7.5, -5.5): 12,    #"vermelha_baixo_12"
            (8, 10, -5.5, -3.5): 13,    #"vermelha_baixo_13"
            (8, 10, -3.5, -1.5): 14,    #"vermelha_baixo_14"
            (8, 10, -1.5, 0.5): 15,     #"vermelha_baixo_15"

            # Vermelha (Cima)
            (-10, -8, 0.5, 2.5): 16,    #"vermelha_cima_0"
            (-10, -8, 2.5, 4.5): 17,    #"vermelha_cima_1"
            (-10, -8, 4.5, 6.5): 18,    #"vermelha_cima_2"
            (-8, -6, 4.5, 6.5): 19,     #"vermelha_cima_4"
            (-6, -4, 4.5, 6.5): 20,     #"vermelha_cima_5"
            (-4, -2, 4.5, 6.5): 21,     #"vermelha_cima_6"
            (-2, 0, 4.5, 6.5): 22,      #"vermelha_cima_7"
            (0, 2, 4.5, 6.5): 23,       #"vermelha_cima_8"
            (2, 4, 4.5, 6.5): 24,       #"vermelha_cima_9"
            (4, 6, 4.5, 6.5): 25,       #"vermelha_cima_10"
            (6, 8, 4.5, 6.5): 26,       #"vermelha_cima_11"
            (8, 10, 0.5, 2.5): 27,      #"vermelha_cima_12"
            (8, 10, 2.5, 4.5): 28,      #"vermelha_cima_13"
            (8, 10, 4.5, 6.5): 29,      #"vermelha_cima_14"

            # Amarelo
            (-8, -6, -5.5, -3.5): 30,   #"amarelo"
            (-6, -4, -5.5, -3.5): 31,   #"amarelo"
            (-6, -4, 2.5, 4.5): 32,     #"amarelo"
            (-4, -2, -5.5, -3.5): 33,   #"amarelo"
            (-4, -2, 2.5, 4.5): 34,     #"amarelo"
            (-2, 0, -5.5, -3.5): 35,    #"amarelo"
            (-2, 0, 2.5, 4.5): 36,      #"amarelo"
            (0, 2, -5.5, -3.5): 37,     #"amarelo"
            (0, 2, 2.5, 4.5): 38,       #"amarelo"
            (2, 4, -5.5, -3.5): 39,     #"amarelo"
            (2, 4, 2.5, 4.5): 40,       #"amarelo"
            (4, 6, -5.5, -3.5): 41,     #"amarelo"
            (4, 6, 2.5, 4.5): 42,       #"amarelo"
            (6, 8, -5.5, -3.5): 43,     #"amarelo"
            (6, 8, -3.5, -1.5): 44,     #"amarelo"
            (6, 8, -1.5, 0.5): 45,      #"amarelo"
            (6, 8, 0.5, 2.5): 46,       #"amarelo"
            (6, 8, 2.5, 4.5): 47,       #"amarelo"

            # Verde
            (-6, -4, -3.5, -1.5): 48,   #"verde"
            (-6, -4, -1.5, 0.5): 49,    #"verde"
            (-4, -2, -3.5, -1.5): 50,   #"verde"
            (-4, -2, -1.5, 0.5): 51,    #"verde"
            (-2, 0, -3.5, -1.5): 52,    #"verde"
            (-2, 0, -1.5, 0.5): 53,     #"verde"
            (0, 2, -3.5, -1.5): 54,     #"verde"
            (0, 2, -1.5, 0.5): 55,      #"verde"
            (2, 4, -3.5, -1.5): 56,     #"verde"
            (2, 4, -1.5, 0.5): 57,      #"verde"
            (4, 6, -3.5, -1.5): 58,     #"verde"
            (4, 6, -1.5, 0.5): 59,      #"verde"
        }


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
        Retorna a cor da região onde o ponto (x, y) está localizado.

        Parâmetros:
            x (float): Coordenada X do ponto.
            y (float): Coordenada Y do ponto.

        Retorna:
            str: Nome da região ou 'fora_do_mapa' se o ponto não pertence a nenhuma região.
        """
        for (x_min, x_max, y_min, y_max), regiao in self.regioes.items():
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return regiao
        return "fora_do_mapa"

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
    print(type(cor))
    print(f"A posição ({x}, {y}) está na região: {cor}")

    # Plotar o mapa
    fig, ax = plt.subplots()
    mapa.desenhar_mapa(ax)
    plt.axis('equal')
    plt.legend()
    plt.show()
