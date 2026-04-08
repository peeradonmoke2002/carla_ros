#!/usr/bin/python3
"""
Get trajectory/path FROM Autoware and extract waypoints for CARLA visualization.

Flow:
  Autoware (Behavior Planner)
    ↓ publishes trajectory
  carla_waypoint_publisher
    ↓ subscribes + extracts waypoints
  CARLA visualization
"""
import carla

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaWorldInfo
from carla_waypoint_types.srv import GetWaypoint, GetActorWaypoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from autoware_planning_msgs.msg import Trajectory


class CarlaWaypointPublisher(CompatibleNode):
    """
    Subscribe to Autoware's trajectory and publish waypoints for CARLA.

    Subscribes to: /planning/scenario_planning/trajectory (or similar)
    Publishes to: /carla/{role_name}/waypoints
    """

    def __init__(self):
        """Constructor"""
        super(CarlaWaypointPublisher, self).__init__('carla_waypoint_publisher')
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.ego_vehicle = None
        self.role_name = self.get_param("role_name", 'ego_vehicle')

        # Waypoint publisher
        self.waypoint_publisher = self.new_publisher(
            Path,
            '/carla/{}/waypoints'.format(self.role_name),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.loginfo(f"Publishing waypoints to /carla/{self.role_name}/waypoints")

        # Services
        self.get_waypoint_service = self.new_service(
            GetWaypoint,
            '/carla_waypoint_publisher/{}/get_waypoint'.format(self.role_name),
            self.get_waypoint)
        self.get_actor_waypoint_service = self.new_service(
            GetActorWaypoint,
            '/carla_waypoint_publisher/{}/get_actor_waypoint'.format(self.role_name),
            self.get_actor_waypoint)

        # Subscribe to Autoware trajectory
        self.trajectory_subscriber = self.new_subscription(
            Trajectory,
            "/planning/scenario_planning/trajectory",
            self.on_trajectory,
            qos_profile=10)
        self.loginfo("Subscribed to /planning/scenario_planning/trajectory")

        # Ego vehicle finder
        self.on_tick = None
        self.loginfo("Waiting for ego vehicle...")
        self.on_tick = self.world.on_tick(self.find_ego_vehicle_actor)

    def destroy(self):
        """Destructor"""
        self.ego_vehicle = None
        if self.on_tick:
            self.world.remove_on_tick(self.on_tick)

    def get_waypoint(self, req, response=None):
        """Get the waypoint for a location"""
        carla_position = carla.Location()
        carla_position.x = req.location.x
        carla_position.y = -req.location.y
        carla_position.z = req.location.z

        carla_waypoint = self.map.get_waypoint(carla_position)

        response = roscomp.get_service_response(GetWaypoint)
        response.waypoint.pose = trans.carla_transform_to_ros_pose(carla_waypoint.transform)
        response.waypoint.is_junction = carla_waypoint.is_junction
        response.waypoint.road_id = carla_waypoint.road_id
        response.waypoint.section_id = carla_waypoint.section_id
        response.waypoint.lane_id = carla_waypoint.lane_id
        return response

    def get_actor_waypoint(self, req, response=None):
        """Convenience method to get the waypoint for an actor"""
        actor = self.world.get_actors().find(req.id)

        response = roscomp.get_service_response(GetActorWaypoint)
        if actor:
            carla_waypoint = self.map.get_waypoint(actor.get_location())
            response.waypoint.pose = trans.carla_transform_to_ros_pose(carla_waypoint.transform)
            response.waypoint.is_junction = carla_waypoint.is_junction
            response.waypoint.road_id = carla_waypoint.road_id
            response.waypoint.section_id = carla_waypoint.section_id
            response.waypoint.lane_id = carla_waypoint.lane_id
        else:
            self.logwarn("get_actor_waypoint(): Actor {} not valid.".format(req.id))
        return response

    def on_trajectory(self, trajectory_msg):
        """Callback: receive trajectory from Autoware and extract waypoints"""
        if not trajectory_msg.points or len(trajectory_msg.points) == 0:
            self.logwarn("Received empty trajectory")
            return

        self.loginfo(f"✓ Received trajectory with {len(trajectory_msg.points)} points")

        # Publish as ROS Path message (for RViz)
        self.publish_waypoints_from_trajectory(trajectory_msg)

        # Draw in CARLA world (for visualization in simulator)
        self.draw_waypoints_in_carla(trajectory_msg)

    def find_ego_vehicle_actor(self, _):
        """Look for ego vehicle"""
        hero = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                hero = actor
                break

        if hero is not None and self.ego_vehicle is None:
            self.loginfo("✓ Ego vehicle found")
            self.ego_vehicle = hero

    def draw_waypoints_in_carla(self, trajectory_msg):
        """Draw waypoints in CARLA world as visual markers"""
        try:
            if not trajectory_msg.points or len(trajectory_msg.points) < 2:
                return

            # Downsample for drawing (every Nth point)
            downsample_factor = max(1, len(trajectory_msg.points) // 12)  # Max 12 points (sparse like TUM)

            prev_location = None

            for i, traj_point in enumerate(trajectory_msg.points):
                if i % downsample_factor != 0:
                    continue

                x = traj_point.pose.position.x
                y = -traj_point.pose.position.y  # CARLA uses negative Y
                z = traj_point.pose.position.z + 0.5  # Lift up for visibility

                location = carla.Location(x, y, z)

                # Draw waypoint as red sphere (small like scenario_runner)
                self.world.debug.draw_point(
                    location,
                    size=0.1,
                    color=carla.Color(255, 0, 0),  # Red
                    life_time=0.1
                )

                # Draw line connecting to previous waypoint
                if prev_location is not None:
                    self.world.debug.draw_line(
                        prev_location,
                        location,
                        color=carla.Color(255, 255, 0),  # Yellow line
                        life_time=0.1,
                        thickness=0.02
                    )

                prev_location = location

            # Always draw the final waypoint
            if len(trajectory_msg.points) > 0:
                final_point = trajectory_msg.points[-1]
                x = final_point.pose.position.x
                y = -final_point.pose.position.y
                z = final_point.pose.position.z + 0.5

                final_location = carla.Location(x, y, z)
                self.world.debug.draw_point(
                    final_location,
                    size=0.2,
                    color=carla.Color(0, 255, 0),  # Green for final point
                    life_time=0.1
                )

                if prev_location is not None:
                    self.world.debug.draw_line(
                        prev_location,
                        final_location,
                        color=carla.Color(255, 255, 0),
                        life_time=0.1,
                        thickness=0.02
                    )

            self.loginfo(f"Drew waypoints in CARLA world (downsampled to ~{len(trajectory_msg.points)//downsample_factor} points)")

        except Exception as e:
            self.logwarn(f"Failed to draw waypoints in CARLA: {e}")

    def publish_waypoints_from_trajectory(self, trajectory_msg):
        """
        Extract waypoints from Autoware trajectory and publish as Path.

        Trajectory points → downsample → publish as Path message
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)

        # Extract waypoints from trajectory points
        # Downsample: keep every Nth point to reduce density
        downsample_factor = max(1, len(trajectory_msg.points) // 15)  # Max 15 waypoints

        for i, traj_point in enumerate(trajectory_msg.points):
            if i % downsample_factor != 0:
                continue

            pose = PoseStamped()
            pose.header.frame_id = "map"

            # Extract position and orientation from trajectory point
            pose.pose.position.x = traj_point.pose.position.x
            pose.pose.position.y = traj_point.pose.position.y
            pose.pose.position.z = traj_point.pose.position.z

            pose.pose.orientation.x = traj_point.pose.orientation.x
            pose.pose.orientation.y = traj_point.pose.orientation.y
            pose.pose.orientation.z = traj_point.pose.orientation.z
            pose.pose.orientation.w = traj_point.pose.orientation.w

            msg.poses.append(pose)

        # Always include final point
        if len(trajectory_msg.points) > 0:
            final_point = trajectory_msg.points[-1]
            if len(msg.poses) == 0 or (
                msg.poses[-1].pose.position.x != final_point.pose.position.x or
                msg.poses[-1].pose.position.y != final_point.pose.position.y
            ):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = final_point.pose.position.x
                pose.pose.position.y = final_point.pose.position.y
                pose.pose.position.z = final_point.pose.position.z
                pose.pose.orientation = final_point.pose.orientation
                msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        self.loginfo(f"Published {len(msg.poses)} waypoints (downsampled from {len(trajectory_msg.points)})")

    def connect_to_carla(self):
        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self.wait_for_message(
                "/carla/world_info",
                CarlaWorldInfo,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                timeout=15.0)
        except ROSException as e:
            self.logerr("Error while waiting for world info: {}".format(e))
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 10)
        self.loginfo(f"CARLA world available. Connecting to {host}:{port}")

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            self.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        self.loginfo("✓ Connected to Carla")


def main(args=None):
    """main function"""
    roscomp.init('carla_waypoint_publisher', args)

    publisher = None
    try:
        publisher = CarlaWaypointPublisher()
        publisher.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if publisher:
            publisher.destroy()
        roscomp.shutdown()


if __name__ == "__main__":
    main()
