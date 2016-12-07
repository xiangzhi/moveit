import rospy

from moveit_msgs.msg import Human

from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from exception import MoveItCommanderException
from moveit_ros_planning_interface import _moveit_planning_scene_interface


class HumanInterface(object):

    def __init__(self):
        """ Create a planning scene interface; it uses both C++ wrapped methods and scene manipulation topics. """
        #self._psi = _moveit_planning_scene_interface.PlanningSceneInterface()
        self._pub_human = rospy.Publisher('/human_object',Human,queue_size=100)
        rospy.sleep(1)

    def add_human(self,id_,head_pose, sight_dir, FOV, size):
        ho = Human()
        ho.operation = Human.ADD
        ho.id = id_
        ho.head_pose = head_pose
        ho.sight_dir = sight_dir
        ho.FOV = FOV
        self._pub_human.publish(ho);
        # rospy.spin()