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
            (-8, -6, -7.5, -5.5): 1,    #"vermelha_baixo_1"  
            (-6, -4, -7.5, -5.5): 2,    #"vermelha_baixo_2" 
            (-4, -2, -7.5, -5.5): 3,    #"vermelha_baixo_3" 
            (-2, 0, -7.5, -5.5): 4,     #"vermelha_baixo_4"              
            (0, 2, -7.5, -5.5): 5,      #"vermelha_baixo_5"
            (2, 4, -7.5, -5.5): 6,      #"vermelha_baixo_6"
            (4, 6, -7.5, -5.5): 7,     #"vermelha_baixo_7"
            (6, 8, -7.5, -5.5): 8,     #"vermelha_baixo_8"
            (8, 10, -7.5, -5.5): 9,    #"vermelha_baixo_9"
            # vermelho esquerda
            (-10, -8, -5.5, -3.5): 10,   #"vermelha_esquerda_10"
            (-10, -8, -3.5, -1.5): 11,   #"vermelha_esquerda_11"
            (-10, -8, -1.5, 0.5): 12,    #"vermelha_esquerda_12"
            (-10, -8, 0.5, 2.5): 13,    #"vermelha_esquerda_13"
            (-10, -8, 2.5, 4.5): 14,    #"vermelha_esquerda_14"
            (-10, -8, 4.5, 6.5): 15,    #"vermelha_esquerda_15"
            # Vermelha Cima
            (-10, -8, 6.5, 8.5): 16,     #"vermelha_cima_16"
            (-8, -6, 6.5, 8.5): 17,      #"vermelha_cima_17
            (-6, -4, 6.5, 8.5): 18,      #"vermelha_cima_18"
            (-4, -2, 6.5, 8.5): 19,      #"vermelha_cima_19"
            (-2, 0, 6.5, 8.5): 20,       #"vermelha_cima_20"
            (0, 2, 6.5, 8.5): 21,       #"vermelha_cima_21"
            (2, 4, 6.5, 8.5): 22,       #"vermelha_cima_22"
            (4, 6, 6.5, 8.5): 23,       #"vermelha_cima_23"
            (6, 8, 6.5, 8.5): 24,       #"vermelha_cima_24"
            (8, 10, 6.5, 8.5): 25,      #"vermelha_cima_25"
            # vermelho direita
            (8, 10, -5.5, -3.5): 26,     #"vermelha_direita_26"
            (8, 10, -3.5, -1.5): 27,    #"vermelha_direita_27"
            (8, 10, -1.5, 0.5): 28,     #"vermelha_direita_28"
            (8, 10, 0.5, 2.5): 29,      #"vermelha_direita_29"
            (8, 10, 2.5, 4.5): 30,      #"vermelha_direita_30"
            (8, 10, 4.5, 6.5): 31,      #"vermelha_direita_31"          
            
            # Amarelo baixo
            (-8, -6, -5.5, -3.5): 32,   #"amarelo"
            (-6, -4, -5.5, -3.5): 33,   #"amarelo"
            (-4, -2, -5.5, -3.5): 34,   #"amarelo"
            (-2, 0, -5.5, -3.5): 35,    #"amarelo"
            (0, 2, -5.5, -3.5): 36,     #"amarelo"
            (2, 4, -5.5, -3.5): 37,     #"amarelo"
            (4, 6, -5.5, -3.5): 38,     #"amarelo"
            (6, 8, -5.5, -3.5): 39,     #"amarelo" 
            #amarelo esquerda  
            (-8, -6, -3.5, -1.5): 40,    #"amarelo"
            (-8, -6, -1.5, 0.5): 41,     #"amarelo"
            (-8, -6, 0.5, 2.5): 42,      #"amarelo"
            (-8, -6, 2.5, 4.5): 43,      #"amarelo"            
            #amarelo cima
            (-8, -6, 4.5, 6.5): 44,      #"amarelo"
            (-6, -4, 4.5, 6.5): 45,     #"amarelo"
            (-4, -2, 4.5, 6.5): 46,     #"amarelo"
            (-2, 0, 4.5, 6.5): 47,      #"amarelo"
            (0, 2, 4.5, 6.5): 48,       #"amarelo"
            (2, 4, 4.5, 6.5): 49,       #"amarelo"
            (4, 6, 4.5, 6.5): 50,       #"amarelo"
            (6, 8, 4.5, 6.5): 51,       #"amarelo"
            #amarelo direita
            (6, 8, 2.5, 4.5): 52,       #"amarelo"
            (6, 8, 0.5, 2.5): 53,       #"amarelo"
            (6, 8, -1.5, 0.5): 54,      #"amarelo"
            (6, 8, -3.5, -1.5): 55,     #"amarelo"
             
            # Verde baixo
            (-6, -4, -3.5, -1.5): 56,   #"verde"
            (-4, -2, -3.5, -1.5): 57,   #"verde"
            (-2, 0, -3.5, -1.5): 58,    #"verde"
            (0, 2, -3.5, -1.5): 59,     #"verde"
            (2, 4, -3.5, -1.5): 60,     #"verde"
            (4, 6, -3.5, -1.5): 61,     #"verde"
            #verde esquerda
            (-6, -4, -1.5, 0.5): 62,    #"verde"
            (-6, -4, 0.5, 2.5): 63,    #"verde"
            #verde cima
            (-6, -4, 2.5, 4.5): 64,    #"verde"
            (-4, -2, 2.5, 4.5): 65,    #"verde"
            (-2, 0, 2.5, 4.5): 66,    #"verde"
            (0, 2, 2.5, 4.5): 67,    #"verde"
            (2, 4, 2.5, 4.5): 68,    #"verde"
            (4, 6, 2.5, 4.5): 69,    #"verde"
            #verde direita
            (4, 6, 0.5, 2.5): 70,    #"verde"
            (4, 6, -1.5, 0.5): 71,    #"verde"
            (-4, -2, -1.5, 0.5): 72,
            (-2, 0, -1.5, 0.5): 73,
            (0, 2, -1.5, 0.5): 74,
            (2, 4, -1.5, 0.5): 75,
            (-4, -2, 0.5, 2.5): 76,
            (-2, 0, 0.5, 2.5): 77,
            (0, 2, 0.5, 2.5): 78,
            (2, 4, 0.5, 2.5): 79

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

    def obter_cor_regiao(self, x, y, tolerancia=1e-6):
        """
        Retorna a cor da região onde o ponto (x, y) está localizado.
        Inclui depuração para verificar os valores.

        Parâmetros:
            x (float): Coordenada X do ponto.
            y (float): Coordenada Y do ponto.
            tolerancia (float): Margem de erro para comparar floats.

        Retorna:
            int: Identificador da região ou 500 se o ponto não estiver em nenhuma região.
        """
        for (x_min, x_max, y_min, y_max), regiao in self.regioes.items():
            #print(f"Verificando região: ({x_min}, {x_max}, {y_min}, {y_max}) para ponto ({x}, {y})")
            if (x_min - tolerancia) <= x <= (x_max + tolerancia) and (y_min - tolerancia) <= y <= (y_max + tolerancia):
                #print(f"Ponto ({x}, {y}) encontrado na região {regiao}")
                return regiao
        #print(f"Ponto ({x}, {y}) não encontrado em nenhuma região.")
        return 500

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
    #x, y = 1.7732411925348928, 0.6024850223445085
    #x, y = -9.8464545465, -7.34534535
    x,y = 5.06, 5.50
    cor = mapa.obter_cor_regiao(round(x,2), round(y,2))
    # Plotar o mapa
    fig, ax = plt.subplots()
    ax.plot(x, y, 'o', color='blue', label=f'Ponto ({x}, {y})')
    mapa.desenhar_mapa(ax)
    plt.axis('equal')
    plt.legend()
    plt.show()
