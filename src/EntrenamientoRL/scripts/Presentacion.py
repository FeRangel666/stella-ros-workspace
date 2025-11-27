from std_msgs.msg import String
import rospy
from std_msgs.msg import Int32


class ControlRobot:
    def __init__(self):
        rospy.init_node('Nodo_publisher_shido')
        rospy.loginfo("[PUBLISHER  PY  NODE] " + rospy.get_namespace())

        self.ini_pose = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=10)
        self.module = rospy.Publisher("/robotis/enable_ctrl_module", String, queue_size=10)
        self.action_page_num = rospy.Publisher("/robotis/action/page_num", Int32, queue_size=10)

        # Subscriber for the "movement_done" topic
        self.movement_done_subscriber = rospy.Subscriber("/robotis/movement_done", String, self.movement_done_callback)
        self.movement_done = False  # Initialize the flag

    def movement_done_callback(self, msg):
        rospy.loginfo("Se completó la acción!: {}".format(msg.data))
        self.movement_done = True  # Set the flag to True when movement is done

    def wait_for_duration(self, duration):
        rospy.sleep(duration)

    def publish_ini_pose(self):
        self.ini_pose.publish('ini_pose')
        self.wait_for_duration(5)

    def put_robot_in_mode(self, mode):
        self.module.publish(mode)
        self.wait_for_duration(3)

    def publish_action_page_num(self, page_num):
        self.action_page_num.publish(page_num)
        # self.wait_for_duration(3)

    def publicar(self, tipo_pose: str) -> None:
        if tipo_pose == 'pose_inicial':
            self.publish_ini_pose()
            self.put_robot_in_mode('none')
            self.put_robot_in_mode('action_module')

            actions = [1, 3, 4, 3, 23, 24, 38]

            while True:
                for action in actions:
                    self.publish_action_page_num(action)

                    # Wait for the completion signal before moving to the next action
                    while not rospy.is_shutdown():
                        if self.movement_done:
                            break
                        rospy.sleep(1)

                    # Reset the movement_done flag for the next action
                    self.movement_done = False

                    # Wait for 2 seconds before moving to the next action
                    # self.wait_for_duration(2)
        else:
            rospy.loginfo("Tipo de pose no reconocido")


# Initialize the ROS node and ControlRobot class
rospy.sleep(1)
robot = ControlRobot()

# Publicar la pose inicial del robot
rospy.sleep(3)
robot.publicar('pose_inicial')

