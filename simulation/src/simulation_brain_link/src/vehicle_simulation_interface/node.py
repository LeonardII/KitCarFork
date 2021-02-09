from contextlib import suppress

import geometry_msgs.msg
import rospy
import std_msgs.msg
from simulation_brain_link.msg import State as StateEstimationMsg
from tf2_msgs.msg import TFMessage

from simulation.utils.geometry import Pose, Transform, Vector
from simulation.utils.ros_base.node_base import NodeBase


class VehicleSimulationInterfaceNode(NodeBase):
    def __init__(self):
        self.speed = Vector(1, 0)
        self.yaw_rate = 0

        super().__init__(
            name="vehicle_simulation_interface_node"
        )  # Name can be overwritten in launch file

        self.run(function=self.update, rate=float(24))

    def start(self):

        self.target_speed_subscriber = rospy.Subscriber(
            "/targetSpeed",
            std_msgs.msg.Int16,
            callback=self.receive_target_speed,
            queue_size=1,
        )
        self.target_steering_angle_subscriber = rospy.Subscriber(
            "/targetSteeringAngle",
            std_msgs.msg.Int16,
            callback=self.receive_target_steering_angle,
            queue_size=1,
        )
        self.state_estimation_publisher = rospy.Publisher(
            "/control_debug/vehicle_simulation/true_state",
            StateEstimationMsg,
            self.update_state_estimation,
            queue_size=1,
        )
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=100)

        # Read initial position from vehicle simulation link parameters
        try:
            initial = [0.4, -0.2]  # self.param.vehicle_simulation_link.initial_pose
            if len(initial) > 3:
                angle = initial[3]
                del initial[3]
            else:
                angle = 0
            pos = Vector(initial)
            self.initial_tf = Transform(pos, angle)
            self.position = pos
            self.angle = angle
        except KeyError:
            self.initial_tf = None
            self.position = Vector(0, 0)
            self.angle = 0

        super().start()

    def update(self):
        self.position = self.position + (1 / 24) * self.speed
        self.angle = self.yaw_rate
        pose = Pose(
            self.position,
            self.angle,
        )
        print("update position: ", self.position, self.angle)
        self.update_world_vehicle_tf(
            self.initial_tf.inverse * Transform(pose, pose.get_angle())
        )

    def stop(self):
        # Attribute errors can occur if the node has not been completely started
        # before shutting down.
        with suppress(AttributeError):
            self.target_speed_subscriber.unregister()
            self.target_steering_angle_subscriber.unregister()
            self.state_estimation_publisher.unregister()
            self.pub_tf.unregister()
        super().stop()

    def receive_target_steering_angle(self, msg: std_msgs.msg.Int16):
        print("received angle:", msg.data)
        self.yaw_rate = msg.data

    def receive_target_speed(self, msg: std_msgs.msg.Int16):
        print("received speed:", msg.data)
        self.speed.x = msg.data

    def update_state_estimation(self):
        msg = StateEstimationMsg()
        msg.speed_x = self.speed.x
        msg.speed_y = self.speed.y
        msg.yaw_rate = self.yaw_rate
        self.state_estimation_publisher.publish(msg)

    def update_world_vehicle_tf(self, vehicle_world_tf: Transform):
        """Publish up to date world to vehicle transformation to /tf.

        Args:
            vehicle_world_tf(Transform): Transformation between vehicle and world frames.
        """
        tf_stamped = geometry_msgs.msg.TransformStamped()

        tf_stamped.header = std_msgs.msg.Header()
        tf_stamped.header.stamp = rospy.Time.now()
        # Transform from world to vehicle
        tf_stamped.header.frame_id = (
            "odom"  # self.param.vehicle_simulation_link.frame.world
        )
        tf_stamped.child_frame_id = (
            "vehicle"  # self.param.vehicle_simulation_link.frame.vehicle
        )

        # Transformation from world to vehicle
        tf_stamped.transform = (vehicle_world_tf).to_geometry_msg()

        self.pub_tf.publish(TFMessage([tf_stamped]))
