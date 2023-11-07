#!/usr/bin/env python3
from typing import List, Optional
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from rclpy.parameter import Parameter
from pathlib import Path
from datetime import datetime
import csv

class ClickedPointWaypointLogger2D(Node):
    """ROS node that writes a sequence of waypoints published to the
    /clicked_point topic to a file. Intedned use case is using this to create a
    rough path in RVIZ by repeatedly creating new points with RVIZ's publish
    point button.
    """
    """Default directory where the waypoint file will be created at.
    """
    __default_directory = "/home/tianhao/sim_ws/src/f1tenth_lab6/lab6_pkg/waypoint"

    def __init__(self, *args, directory: Optional[str] = None, **kwargs) -> None:
        """Creates a new ClickedPointWaypointLogger2D instance.

        Args:
            directory (Optional[str], optional): Directory where the logged
            waypoints file will be written to. If not specified, will be written
            to the current/executing user's home directory.
        """
        super().__init__(*args, node_name="waypoint_logger", **kwargs)
        # Reminder: In a function call, * and ** unpack the data structures into
        # arguments to be passed into the function. In a function definition
        # (def), * and ** are used to indicate that you want all arguments and
        # keyword arguments to be packed into a tuple and dictionary,
        # respectively.

        # Open a new waypoints file in the user's home directory--or the
        # specified directory in constructor, if one provided.
        self.__directory = directory
        if self.__directory == None:
            self.__directory = ClickedPointWaypointLogger2D.__default_directory
        self.__default_filename = f"{datetime.now()}_waypoints"

        # Generate a new, unique file name within the default directory.
        def unique_path(directory: Path, file_name: str, extension: str) -> Path:
            path = Path(directory, f"{file_name}.{extension}")
            if path.exists():
                counter = 1
                unique_name_found = False
                while not unique_name_found:
                    path = Path(directory, f"{file_name}_{counter}.{extension}")
                    if path.exists():
                        counter += 1
            return path
        
        # Generate a unique filename for the waypoints file.
        self.__waypoints_filepath = unique_path(directory=self.__directory,
                                                file_name=self.__default_filename,
                                                extension="csv")

        try:
            self.__waypoint_file = open(self.__waypoints_filepath, 'w')
        except OSError as exc:
            self.get_logger().error(f"Failed to open file at filepath {self.__waypoints_filepath}")
            raise exc
 
        self.get_logger().info(f"Successfully created waypoint logging file {self.__waypoints_filepath}")
        self.__csv_writer = csv.writer(self.__waypoint_file)

        # Create the subscriber that will subscribe to the clicked_point topic.
        self.create_subscription(msg_type=PointStamped,
                                 topic="/clicked_point",
                                 callback=self.__clicked_point_callback,
                                 qos_profile=10)

        self.get_logger().info(f"ClickedPointWaypointLogger2D successfully created.")

    def __clicked_point_callback(self, new_waypoint: PointStamped) -> None:
        """Take the provided waypoint and write it to a new line in the current
        waypoints file.

        Args:
            new_waypoint (PointStamped): New waypoint to be recorded.
        """
        self.__csv_writer.writerow([new_waypoint.point.x, new_waypoint.point.y])
        self.get_logger().info(f"Wrote new 2D waypoint {[new_waypoint.point.x, new_waypoint.point.y]} to file.")

    def destroy_node(self) -> bool:
        """Default implementation for destroying node, but also closes the
        waypoints file.
        """
        self.__waypoint_file.close()
        self.get_logger().info(f"Waypoint logger saved waypoint file {self.__waypoints_filepath}.")
        return super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = ClickedPointWaypointLogger2D()
    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
