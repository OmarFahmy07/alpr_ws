import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time

from alpr_bot.utils.gps_utils import latLonYaw2Geopose


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)
            self.gepose_wps = []
            for wp in self.wps_dict["waypoints"]:
                latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
                self.gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        return self.gepose_wps


class GpsWpCommander():
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        num_loops = 0
        while True:
            self.navigator.waitUntilNav2Active(localizer='robot_localization')
            wps = self.wp_parser.get_wps()
            self.navigator.followGpsWaypoints(wps)
            while (not self.navigator.isTaskComplete()):
                time.sleep(0.1)
            num_loops+=1
            print("wps completed successfully, ",num_loops )
            time.sleep(5)


def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "alpr_bot"), "config", "demo_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()
