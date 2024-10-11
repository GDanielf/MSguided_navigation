#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from guided_navigation.msg import PoseEstimate
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
import numpy as np

class FiltroParticulas(Node):
    def __init__(self):
        super().__init__('filtro_particulas')

        # Subscriber para o tópico /data_topic
        self.subscription = self.create_subscription(
            PoseEstimate,
            '/pose_estimate',
            self.filtro_callback,
            10
        )

        # Configuração do gráfico no PyQtGraph
        self.x_data = np.array([])
        self.y_data = np.array([])

        self.app = QtWidgets.QApplication([])

        self.win = pg.GraphicsLayoutWidget(show=True)
        self.plot = self.win.addPlot()
        self.curve = self.plot.plot(pen='w')

        # Timer para atualizar o gráfico
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # Atualiza a cada 100ms

    def filtro_callback(self, msg):
        # Adiciona o dado recebido aos arrays x_data e y_data
        self.x_data = np.append(msg.x)
        self.y_data = np.append(msg.y)
        print(f'Received: {msg.x, msg.y}')

    def update_plot(self):
        # Atualiza o gráfico com os novos dados
        self.curve.setData(self.x_data, self.y_data)

    def run(self):
        # Inicia a aplicação PyQtGraph
        self.app.exec_()


def main(args=None):
    rclpy.init(args=args)
    node = FiltroParticulas()
    try:
        node.run()  # Executa o node e a interface gráfica
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
