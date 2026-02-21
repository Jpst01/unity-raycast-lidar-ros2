import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import threading
import json
import math
import time
from collections import deque


class LidarBridgeNode(Node):
    def __init__(self):
        super().__init__('lidar_bridge_node')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 9090)
        self.declare_parameter('frame_id', 'lidar_frame')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.msg_count = 0

        self.scan_queue = deque(maxlen=5)
        self.queue_lock = threading.Lock()

        self.publish_timer = self.create_timer(0.02, self.publish_from_queue)

        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f'LiDAR Bridge started — listening on {self.host}:{self.port}'
        )

    def publish_from_queue(self):
        with self.queue_lock:
            while self.scan_queue:
                msg = self.scan_queue.popleft()
                self.scan_pub.publish(msg)
                self.msg_count += 1
                if self.msg_count <= 5 or self.msg_count % 100 == 0:
                    num_ranges = len(msg.ranges)
                    self.get_logger().info(
                        f'Published scan #{self.msg_count} with {num_ranges} rays'
                    )

    def run_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        self.get_logger().info('Waiting for Unity connection...')

        while rclpy.ok():
            try:
                conn, addr = server.accept()
                self.get_logger().info(f'Unity connected from {addr}')
                self.handle_client(conn)
            except Exception as e:
                self.get_logger().error(f'Server error: {e}')
                time.sleep(1.0)

    def handle_client(self, conn):
        buffer = ''
        try:
            while rclpy.ok():
                data = conn.recv(65536)
                if not data:
                    self.get_logger().warn('Unity disconnected')
                    break

                buffer += data.decode('utf-8')

                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue

                    try:
                        scan_data = json.loads(line)
                        self.enqueue_scan(scan_data)
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f'JSON parse error: {e}')
        except Exception as e:
            self.get_logger().error(f'Client error: {e}')
        finally:
            conn.close()
            self.get_logger().info('Client connection closed, waiting for new connection...')

    def enqueue_scan(self, scan_data):
        msg = LaserScan()

        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = float(scan_data.get('angle_min', 0.0))
        msg.angle_max = float(scan_data.get('angle_max', 2.0 * math.pi))
        msg.angle_increment = float(scan_data.get('angle_increment', 2.0 * math.pi / 360.0))
        msg.range_min = float(scan_data.get('range_min', 0.1))
        msg.range_max = float(scan_data.get('range_max', 20.0))

        ranges = scan_data.get('ranges', [])
        msg.ranges = [float(r) for r in ranges]

        msg.time_increment = 0.0
        msg.scan_time = 1.0 / 10.0
        msg.intensities = []

        with self.queue_lock:
            self.scan_queue.append(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
