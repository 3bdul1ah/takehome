#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math, random
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from PIL import Image
import yaml

class SimpleRRT(Node):
    def __init__(self):
        super().__init__('simplerrt_node')
        self.yaml_map_path = '/home/abdullah/dev_ws/src/rrt/map/map.yaml'   # change the path
        self.step_size = 0.2
        self.goal_bias = 0.2
        self.max_iter = 1000
        self.edge_resolution = 0.05
        self.map_data = self.load_map(self.yaml_map_path)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.path_pub = self.create_publisher(Path, '/rrt_path', 10)
        self.start_marker_pub = self.create_publisher(Marker, '/start_marker', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.start_sub = self.create_subscription(PoseStamped, '/start_pose', self.start_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.start = PoseStamped()
        self.start.pose.position.x = -2.0
        self.start.pose.position.y = 0.6
        self.goal = PoseStamped()
        self.goal.pose.position.x = 2.1
        self.goal.pose.position.y = -1.4
        self.path_found = False
        self.create_timer(1.0, self.publish_map_and_plan)

    def load_map(self, yaml_file):
        with open(yaml_file) as f:
            map_yaml = yaml.safe_load(f)
        img = Image.open(map_yaml['image']).convert('L')
        width, height = img.size
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = width
        grid.info.height = height
        grid.info.resolution = map_yaml['resolution']
        grid.info.origin.position.x = map_yaml['origin'][0]
        grid.info.origin.position.y = map_yaml['origin'][1]
        grid.data = [100 if img.getpixel((x, height - 1 - y)) < 127 else 0
                     for y in range(height) for x in range(width)]
        return grid

    def start_callback(self, msg: PoseStamped):
        self.start = msg
        self.path_found = False

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg
        self.path_found = False

    def is_free(self, x, y):
        mx = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        if mx < 0 or my < 0 or mx >= self.map_data.info.width or my >= self.map_data.info.height:
            return False
        return self.map_data.data[my * self.map_data.info.width + mx] == 0

    def is_path_free(self, x1, y1, x2, y2):
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = int(dist / self.edge_resolution)
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not self.is_free(x, y):
                return False
        return True

    def publish_marker(self, pose, pub, color):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        pub.publish(marker)

    def publish_map_and_plan(self):
        self.map_data.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_data)
        self.publish_marker(self.start, self.start_marker_pub, (1.0, 0.0, 0.0))
        self.publish_marker(self.goal, self.goal_marker_pub, (0.0, 1.0, 0.0))
        if not self.path_found:
            path_points = self.rrt(self.start.pose.position, self.goal.pose.position)
            if path_points:
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                path_msg.header.stamp = self.get_clock().now().to_msg()
                for x, y in path_points:
                    ps = PoseStamped()
                    ps.pose.position.x = x
                    ps.pose.position.y = y
                    ps.pose.orientation.w = 1.0
                    path_msg.poses.append(ps)
                self.path_pub.publish(path_msg)
                self.path_found = True

    def rrt(self, start, goal):
        tree = [(start.x, start.y)]
        parent = { (start.x, start.y): None }
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        max_x = origin_x + self.map_data.info.width * self.map_data.info.resolution
        max_y = origin_y + self.map_data.info.height * self.map_data.info.resolution
        for _ in range(self.max_iter):
            if random.random() < self.goal_bias:
                x_rand, y_rand = goal.x, goal.y
            else:
                x_rand = random.uniform(origin_x, max_x)
                y_rand = random.uniform(origin_y, max_y)
            nearest = min(tree, key=lambda n: (n[0]-x_rand)**2 + (n[1]-y_rand)**2)
            theta = math.atan2(y_rand - nearest[1], x_rand - nearest[0])
            x_new = nearest[0] + self.step_size * math.cos(theta)
            y_new = nearest[1] + self.step_size * math.sin(theta)
            if self.is_free(x_new, y_new) and self.is_path_free(nearest[0], nearest[1], x_new, y_new):
                tree.append((x_new, y_new))
                parent[(x_new, y_new)] = nearest
                if math.hypot(goal.x - x_new, goal.y - y_new) < self.step_size:
                    if self.is_path_free(x_new, y_new, goal.x, goal.y):
                        parent[(goal.x, goal.y)] = (x_new, y_new)
                        path = []
                        node = (goal.x, goal.y)
                        while node is not None:
                            path.append(node)
                            node = parent.get(node)
                        path.reverse()
                        return path
        return None

def main():
    rclpy.init()
    node = SimpleRRT()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
