# 这个代码暂时废弃，整体效果不如adjust_time.sh好使

# import rclpy
# from std_msgs.msg import String
# import time
# import threading


# def main(args = None):
#     rclpy.init(args = args)

#     node = rclpy.create_node('vehicle_adjust_time')
#     publisher = node.create_publisher(String, '/vehicle/adjust_date', 10)

#     def send():
#         msg = String()

#         time_stamp = time.time() + 2
#         head = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time_stamp))
#         foot = str(time_stamp).split('.')[1]

#         result = head + "." + foot

#         msg.data = result
#         node.get_logger().info('Publishing: "%s"' % msg.data)
    
#         publisher.publish(msg)

#         time.sleep(1)

#         node.destroy_node()
#         rclpy.shutdown()

#     timer = threading.Timer(1.0, send)
#     timer.start()

#     rclpy.spin(node)


# if __name__ == '__main__':
#     main()