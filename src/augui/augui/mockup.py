import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from std_srvs.srv import SetBool
import math
from scipy.spatial.transform import Rotation as R


class PowerOnOff():
    def __init__(self, node, name):
        self.name = name
        self.node = node
        self.onoff_power = Bool()
        self.pub_power = node.create_publisher(Bool, "power/" + name, 10)
        self.srv_power = node.create_service(SetBool, "set_power_" + name, self.cb_onoff)

    def cb_onoff(self, req, res):
        self.node.get_logger().info(
            self.name + " receives a service call with request " + str(req.data))
        self.onoff_power.data = req.data
        res.success = True
        return res

    def publish(self):
        self.pub_power.publish(self.onoff_power)


class Mockup(Node):
    def __init__(self):
        super().__init__('mockup')
        self.lat = self.declare_parameter(
            "origin_lat", 35.056796).get_parameter_value().double_value
        self.lon = self.declare_parameter(
            "origin_lon", 138.765736).get_parameter_value().double_value
        self.declare_parameter("region_height", 100.0)
        self.declare_parameter("region_width", 200.0)
        self.declare_parameter("region_direction", 30.0)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.pub_gps = self.create_publisher(NavSatFix, 'gps', 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_alt = self.create_publisher(Float64, "altitude", 10)
        self.power_names = [
            "thruster",
            "left_upper",
            "left_lower",
            "right_upper",
            "right_lower",
            "ins",
            "dvl",
            "gps",
            "fls"
        ]
        self.powers = dict()
        for name in self.power_names:
            self.powers.update({name: PowerOnOff(self, name)})
        self.rad = 0.0
        self.odom = Odometry()

    def timer_callback(self):
        self.rad += 0.1
        r = 10.0
        vx = - self.rad * r * math.sin(self.rad)
        vy = self.rad * r * math.cos(self.rad)
        theta = math.atan2(vy, vx)

        # publish odom
        odom = Odometry()
        odom.pose.pose.position.x = r * math.cos(self.rad)
        odom.pose.pose.position.y = r * math.sin(self.rad)
        odom.pose.pose.position.z = 100 * math.sin(self.rad / 100) + 2 * r + r * math.sin(self.rad)
        odom.twist.twist.linear.x = r * self.rad
        odom.twist.twist.angular.z = self.rad
        rpy = R.from_euler('z', theta)
        quat = rpy.as_quat()
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        self.pub_odom.publish(odom)

        # publish gps
        msg = NavSatFix()
        msg.latitude = self.lat + 0.03 * math.cos(self.rad)
        msg.longitude = self.lon + 0.03 * math.sin(self.rad)
        self.pub_gps.publish(msg)

        # publish altitude
        msg = Float64()
        msg.data = 100 * math.cos(self.rad / 100) - (2 * r + r * math.sin(self.rad))
        self.pub_alt.publish(msg)

        # publish power state
        for name in self.power_names:
            self.powers[name].publish()


def main(args=None):
    rclpy.init(args=args)

    mockup = Mockup()

    rclpy.spin(mockup)
    mockup.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
