import os
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

from .planner_factory import planner_factory

class SafePlannerNode(Node):
    def __init__(self):
        super().__init__('safeplan_ros2')
        self.map = None
        self.start = None
        self.goal = None

        # Load planner config
        pkg_share = get_package_share_directory('safeplan_ros2')
        path = os.path.join(pkg_share, 'config', 'algos.yaml')
        self.get_logger().info(f"Loading planner config from: {path}")
        with open(path, 'r') as f:
            config = yaml.safe_load(f)

        algo_name = config['active_algo']
        algo_args = next((a['args'] for a in config['algoDetails'] if a['name'] == algo_name), {})
        self.planner = planner_factory(algo_name, algo_args)
        self.get_logger().info(f"Using planner: {algo_name}")

        # Subscribers
        self.create_subscription(OccupancyGrid, '/binary_map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/start_pose', self.start_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # Publisher
        self.plan_pub = self.create_publisher(Path, '/planner_path', 10)

        # Timer for planning
        self.timer = self.create_timer(1.0, self.plan_loop)

    def map_cb(self, msg):
        self.map = msg

    def start_cb(self, msg):
        self.start = msg.pose

    def goal_cb(self, msg):
        self.goal = msg.pose

    def plan_loop(self):
        if not all([self.map, self.start, self.goal]):
            return

        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height

        try:
            # Convert from world to map coordinates
            start_x = int((self.start.position.x - map_origin_x) / resolution)
            start_y = int((self.start.position.y - map_origin_y) / resolution)
            goal_x = int((self.goal.position.x - map_origin_x) / resolution)
            goal_y = int((self.goal.position.y - map_origin_y) / resolution)

            # Bounds check
            if not (0 <= start_x < width and 0 <= start_y < height):
                self.get_logger().error("Start pose is out of map bounds!")
                return
            if not (0 <= goal_x < width and 0 <= goal_y < height):
                self.get_logger().error("Goal pose is out of map bounds!")
                return

            # Convert map to NumPy grid
            import numpy as np
            grid_data = np.array(self.map.data, dtype=np.int8).reshape((height, width))
            grid_binary = (grid_data >= 1).astype(np.uint8)  # 1=obstacle, 0=free

            start = (start_y, start_x)  # note: (row, col)
            goal = (goal_y, goal_x)

            success, path, info = self.planner.plan(start, goal, grid_binary)

            if not success or not path:
                self.get_logger().warn(f"Planner failed: {info}")
                return

            self.publish_path([(p[1]*resolution + map_origin_x, p[0]*resolution + map_origin_y) for p in path])

        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")



    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.plan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SafePlannerNode())
    rclpy.shutdown()
