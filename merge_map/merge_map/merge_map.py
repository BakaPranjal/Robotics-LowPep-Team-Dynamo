import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import OccupancyGrid, Odometry

class MapMerger(Node):
    def _init_(self):
        super()._init_('map_merger')

        self.map1 = None
        self.map2 = None
        self.odom1 = None
        self.odom2 = None

        self.map_sub1 = self.create_subscription(OccupancyGrid, '/robot1/map', self.map1_callback, 10)
        self.map_sub2 = self.create_subscription(OccupancyGrid, '/robot2/map', self.map2_callback, 10)
        self.odom_sub1 = self.create_subscription(Odometry, '/robot1/odom', self.odom1_callback, 10)
        self.odom_sub2 = self.create_subscription(Odometry, '/robot2/odom', self.odom2_callback, 10)

        self.merged_map_pub = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        self.timer = self.create_timer(1.0, self.merge_maps)  # every second

    def map1_callback(self, msg):
        self.map1 = msg

    def map2_callback(self, msg):
        self.map2 = msg

    def odom1_callback(self, msg):
        self.odom1 = msg

    def odom2_callback(self, msg):
        self.odom2 = msg

    def merge_maps(self):
        if self.map1 is None or self.map2 is None or self.odom1 is None or self.odom2 is None:
            return

        map1 = self.map1
        map2 = self.map2

        width1 = map1.info.width
        height1 = map1.info.height
        resolution = map1.info.resolution

        width2 = map2.info.width
        height2 = map2.info.height

        # Convert map data to numpy arrays
        data1 = np.array(map1.data, dtype=np.int8).reshape((height1, width1))
        data2 = np.array(map2.data, dtype=np.int8).reshape((height2, width2))

        # Compute relative pose (in meters)
        dx = self.odom2.pose.pose.position.x - self.odom1.pose.pose.position.x
        dy = self.odom2.pose.pose.position.y - self.odom1.pose.pose.position.y

        # Convert to pixels
        dx_pix = int(round(dx / resolution))
        dy_pix = int(round(dy / resolution))

        # Determine bounds
        min_x = min(0, dx_pix)
        min_y = min(0, dy_pix)
        max_x = max(width1, dx_pix + width2)
        max_y = max(height1, dy_pix + height2)

        new_width = max_x - min_x
        new_height = max_y - min_y

        # Create blank merged map
        merged = np.full((new_height, new_width), -1, dtype=np.int8)

        # Offsets for placing the maps
        offset1_x = -min_x
        offset1_y = -min_y
        offset2_x = dx_pix + offset1_x
        offset2_y = dy_pix + offset1_y

        # Place map1
        merged[offset1_y:offset1_y + height1, offset1_x:offset1_x + width1] = data1

        # Place map2 with overwrite only for known values
        for y in range(height2):
            for x in range(width2):
                val = data2[y, x]
                if val != -1:
                    mx = x + offset2_x
                    my = y + offset2_y
                    if 0 <= mx < new_width and 0 <= my < new_height:
                        merged[my, mx] = val

        # Prepare OccupancyGrid message
        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = 'map'

        merged_msg.info.resolution = resolution
        merged_msg.info.width = new_width
        merged_msg.info.height = new_height
        merged_msg.info.origin.position.x = map1.info.origin.position.x + (min_x * resolution)
        merged_msg.info.origin.position.y = map1.info.origin.position.y + (min_y * resolution)
        merged_msg.info.origin.position.z = 0.0
        merged_msg.info.origin.orientation.w = 1.0

        # Convert to Python ints
        merged_msg.data = [int(x) for x in merged.flatten()]
        self.merged_map_pub.publish(merged_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
