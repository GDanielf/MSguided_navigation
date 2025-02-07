#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool


class SimulationMonitor(Node):
    def __init__(self):
        super().__init__('simulation_monitor')
        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self.clock_callback, 10)
        self.last_sim_time = None
        self.status_publisher = self.create_publisher(Bool, '/simulation_status', 10)
        self.get_logger().info('Monitor de simulacao inicializado.')   

    def is_simulation_running(self, value):
        status_msg = Bool()
        status_msg.data = value
        self.status_publisher.publish(status_msg)    

    def clock_callback(self, msg):
        current_sim_time = msg.clock
        #print('Current:', msg.clock, 'Last time: ', self.last_sim_time) 
        if self.last_sim_time is not None:
            if current_sim_time == self.last_sim_time:
                self.is_simulation_running(False)
                self.get_logger().info('Simulação está pausada.')
            else:
                self.is_simulation_running(True)
                self.get_logger().info('Simulação está rodando.')
        self.last_sim_time = current_sim_time

def main(args=None):
    rclpy.init(args=args)
    simulation_monitor = SimulationMonitor()
    rclpy.spin(simulation_monitor)
    simulation_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
