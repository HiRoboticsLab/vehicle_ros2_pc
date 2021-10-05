import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import SetParametersResult


class PIDTalker(Node):

    pid_p = 0
    pid_i = 0
    pid_d = 0

    def __init__(self):
        super().__init__('vehicle_adjust_pid')
        self.talker = self.create_publisher(Float64MultiArray, '/vehicle/cmd_pid', 10)


    def callback(self, params):
        for param in params:
            self.get_logger().info(param.name)
            self.get_logger().info(str(param.value))
            self.get_logger().info(str(type(param.value)))

            if(param.name == 'pid_p'):
                self.pid_p = param.value
            if(param.name == 'pid_i'):
                self.pid_i = param.value
            if(param.name == 'pid_d'):
                self.pid_d = param.value

        msg = Float64MultiArray()
        msg.data = [self.pid_p, self.pid_i, self.pid_d]

        self.talker.publish(msg)

        return SetParametersResult(successful = True)



def main(args = None):
    try:
        rclpy.init(args = args)

        talker = PIDTalker()

        talker.declare_parameter('pid_p', 0.1)
        talker.declare_parameter('pid_i', 2.0)
        talker.declare_parameter('pid_d', 0.0001)

        (pid_p, pid_i, pid_d) = talker.get_parameters(
            ['pid_p', 'pid_i', 'pid_d']
        )

        talker.pid_p = pid_p.value
        talker.pid_i = pid_i.value
        talker.pid_d = pid_d.value

        talker.add_on_set_parameters_callback(talker.callback)
        
        executor = SingleThreadedExecutor()
        executor.add_node(talker)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            talker.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()