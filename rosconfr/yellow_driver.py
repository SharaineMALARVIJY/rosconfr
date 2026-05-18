import rclpy
from ackermann_msgs.msg import AckermannDrive
from rclpy.executors import ExternalShutdownException


class YellowDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__vitesse_m_s = 0.0
        self.__direction_degre = 0
        self.__is_active = True
        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('yellow_driver')
        self.__node.create_subscription(AckermannDrive, '/yellow_car/cmd_ackermann', self.__cmd_ackermann_callback
            , 1)
        self.__node.get_logger().info("Noeud créé")
        self.__robot.setCruisingSpeed(0.0) # En km/h
        self.__robot.setSteeringAngle(0.0) # En radian

    def __cmd_ackermann_callback(self, message):
        self.__vitesse_m_s = message.speed
        self.__direction_degre = message.steering_angle
        # self.__node.get_logger().info(
        #     f"[yellow_driver] Reçu : vitesse = {self.__vitesse_m_s} m/s, direction = {self.__direction_degre}°")

    def _shutdown_ros(self):
        if not self.__is_active:
            return
        self.__is_active = False
        self.__robot.setCruisingSpeed(0.0)
        self.__robot.setSteeringAngle(0.0)
        if hasattr(self, '_YellowDriver__node') and self.__node is not None:
            self.__node.destroy_node()
            self.__node = None
        if rclpy.ok():
            rclpy.shutdown()

    def step(self):
        if not self.__is_active or not rclpy.ok():
            self._shutdown_ros()
            return
        try:
            rclpy.spin_once(self.__node, timeout_sec=0)
        except (KeyboardInterrupt, ExternalShutdownException):
            self._shutdown_ros()
            return
        self.__robot.setCruisingSpeed(self.__vitesse_m_s*3.6) # Conversion m/s -> km/h
        self.__robot.setSteeringAngle(-self.__direction_degre*3.14/180) # Conversion degrés -> radians

    def __del__(self):
        self._shutdown_ros()
