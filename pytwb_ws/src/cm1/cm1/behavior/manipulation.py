import py_trees

from pytwb.common import behavior
from lib.actor_bt import ActorBT
from ros_actor import run_actor

@behavior
class Adjust(ActorBT):
    desc = 'adjust arm angle'

    def __init__(self, name, node):
        super().__init__(name, 'ad')

@behavior
class Fit(ActorBT):
    desc = 'adjust arm angle after gripper down'

    def __init__(self, name, node):
        super().__init__(name, 'fit')

@behavior
class Pick(ActorBT):
    desc = 'pick object'

    def __init__(self, name, node):
        super().__init__(name, 
            (
                ('open', None),
                ('pick', None)
            )
        )

@behavior
class Place(ActorBT):
    desc = 'place object'

    def __init__(self, name, node):
        super().__init__(name, 'place')

@behavior
class Open(ActorBT):
    desc = 'gripper open'

    def __init__(self, name, node):
        super().__init__(name, 'open')
        
@behavior
class ArmHome(ActorBT):
    desc = 'set arm home position'

    def __init__(self, name, node):
#        super().__init__(name, 'home')
        super().__init__(name, 
            (
                ('sleep', (1,)),
                ('home', None)
            )
        )
                
# add
@behavior
class Close(ActorBT):
    desc = 'place object'

    def __init__(self, name, node):
        super().__init__(name, 'close')


@behavior
class Home(ActorBT):
    desc = 'arm home'

    def __init__(self, name, node, gripper_anger):
        super().__init__(name, "home", gripper_anger)

@behavior
class PlaceProduct(ActorBT):
    desc = 'place the product'

    def __init__(self, name, node):
        super().__init__(name, "place_product")

@behavior
class MoveToPose(ActorBT):
    desc = 'set arm position'

    def __init__(self, name, diff, flag):
        super().__init__(name, diff, flag)
        self.diff = diff
        self.flag = flag

    def initialise(self):
        bb = py_trees.blackboard.Blackboard()
        self.bb = bb
        base_x = self.bb.get("base_x")
        base_y = self.bb.get("base_y")
        base_z = self.bb.get("base_z")
        self.quat_list = [1, 0, 0, 0]
      
        if self.flag:
            self.quat_list = self.bb.get("quat_list")
        
        run_actor("move_to_pose", position=[base_x, base_y, base_z - self.diff], quat_xyzw = self.quat_list)

    def update(self):
        return py_trees.common.Status.SUCCESS

@behavior 
class EulerToQuat(ActorBT):
    desc = 'euler to quat'

    def __init__(self, name, roll, pitch, yaw):
        super().__init__(name, roll, pitch, yaw)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    
    def initialise(self):
        bb = py_trees.blackboard.Blackboard()
        self.bb = bb
        quat_list = run_actor("euler_to_quat", self.roll, self.pitch, self.yaw)
        bb.set("quat_list", quat_list)
 
    def update(self):
        return py_trees.common.Status.SUCCESS