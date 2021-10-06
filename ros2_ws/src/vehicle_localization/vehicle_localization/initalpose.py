import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import FinishTrajectory
from cartographer_ros_msgs.srv import StartTrajectory

from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory


initalpose = None
client = None


class InitalPose(Node):

    def __init__(self):
        super().__init__('initalpose')
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.callback_pose, 10)


    def callback_pose(self, msg):
        global client
        try:
            self.get_logger().info('rviz set pose: "%s"' % msg)

            result = client.finish_trajectory()
            if result.status.code != 0:
                self.get_logger().error(result.status.message)
            if result.status.code == 0:
                # 这里发现了ros2 cartographer版本问题，待解决
                pass
                # client.start_trajectory(msg.pose.pose)

        except Exception as e:
            self.get_logger().error('%s' % e)


class ServiceClient(Node):

    def __init__(self):
        super().__init__('initalpose_client')
        self.client_finish_trajectory = self.create_client(FinishTrajectory, 'finish_trajectory')
        self.client_start_trajectory = self.create_client(StartTrajectory, 'start_trajectory')
        while not self.client_finish_trajectory.wait_for_service():
            self.get_logger().info('service not available, waiting again...')
        while not self.client_start_trajectory.wait_for_service():
            self.get_logger().info('service not available, waiting again...')
    

    def finish_trajectory(self):
        try:
            request = FinishTrajectory.Request()
            request.trajectory_id = 1
            future = self.client_finish_trajectory.call_async(request)
            
            rclpy.spin_until_future_complete(self, future)

            self.get_logger().info('finish_trajectory service: "%s"' % future.result())

            return future.result()
        except Exception as e:
            self.get_logger().error('%s' % e)


    def start_trajectory(self, pose):
        try:
            request = StartTrajectory.Request()
            self.get_logger().info('%s' % request)

            request.configuration_directory = get_package_share_directory('vehicle_navigation') + '/config'
            request.configuration_basename = 'localization.lua'
            request.use_initial_pose = 1
            request.initial_pose = pose
            request.relative_to_trajectory_id = 1

            future = self.client_start_trajectory.call_async(request)
            
            rclpy.spin_until_future_complete(self, future)

            self.get_logger().info('start_trajectory service: "%s"' % future.result())

            return future.result()
        except Exception as e:
            self.get_logger().error('%s' % e)


def main(args = None):
    global initalpose, client

    rclpy.init(args = args)

    initalpose = InitalPose()
    client = ServiceClient()

    executor = SingleThreadedExecutor()

    executor.add_node(initalpose)
    executor.add_node(client)

    executor.spin()

    executor.shutdown()

    initalpose.destroy_node()
    client.destroy_node()


if __name__ == '__main__':
    main()