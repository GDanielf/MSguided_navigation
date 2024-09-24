import matplotlib.path as mpath

class Mapa:
    def __init__(self):
        """
        Inicializa o mapa com os pontos predefinidos que formam o polígono.
        """
        self.pontos = [
            [-21, 3], [-21, -3], [-11, -3], [-11, -8], [11, -8], [11, -3],
            [21, -3], [21, 3], [11, 3], [11, 8], [-11, 8], [-11, 3]
        ]

    def verifica_ponto_dentro(self, ponto):
        """
        Verifica se um ponto (x, y) está dentro do polígono definido pelos limites do mapa.
        """
        path = mpath.Path(self.pontos)
        return path.contains_point(ponto)

    def desenhar_mapa(self, ax):
        """
        Desenha o mapa representado pelas linhas conectando os pontos no gráfico fornecido.
        """
        # Converte os pontos em duas listas: uma para X e outra para Y
        x_vals = [p[0] for p in self.pontos] + [self.pontos[0][0]]  # Fechando o polígono
        y_vals = [p[1] for p in self.pontos] + [self.pontos[0][1]]
        
        # Desenha o polígono no gráfico
        ax.plot(x_vals, y_vals, 'k-', label='Mapa')

    def __repr__(self):
        return f'MapaComLimites(pontos={self.pontos})'
