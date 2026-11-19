"""point_marker - показать точку из PointStamped в панели 3D.

Панель 3D рисует не любые сообщения: geometry_msgs/PointStamped в её списке
поддерживаемых типов нет, поэтому топик /object_position в настройках панели
не появляется. Эта нода подписывается на него и перекладывает координаты
в visualization_msgs/Marker, который панель 3D рисует.

Запуск из той папки, где лежит файл, сборка не нужна:

    python3 point_marker.py

Затем в панели 3D включите топик /object_marker.
"""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker


class PointMarker(Node):
    def __init__(self):
        super().__init__("point_marker")

        self.declare_parameter("source", "/object_position")
        self.declare_parameter("size", 0.2)
        source = self.get_parameter("source").value
        self.size = float(self.get_parameter("size").value)

        self.pub = self.create_publisher(Marker, "/object_marker", 10)
        self.create_subscription(PointStamped, source, self.point_callback, 10)
        self.get_logger().info(f"читаю {source}, публикую /object_marker")

    def point_callback(self, msg: PointStamped):
        m = Marker()
        m.header = msg.header
        m.ns = "object"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = msg.point
        m.pose.orientation.w = 1.0
        m.scale.x = self.size
        m.scale.y = self.size
        m.scale.z = self.size
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0
        self.pub.publish(m)


def main():
    rclpy.init()
    node = PointMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
