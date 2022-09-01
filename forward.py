import pinocchio as pin 
import numpy as np
from pinocchio.robot_wrapper import RobotWrapper

#change this into the directory that you store the urdf file
urdf_filename = "/home/crrl/pin_ws/src/franka_feedback_linearization_controller/urdf/panda_pin.urdf"; 
model = RobotWrapper.BuildFromURDF(urdf_filename)
controlled_frame = model.model.getFrameId("panda_fingertip"); 
#replace q with the joint angles you want to check 
q = np.array([0,-0.785398163,0,-2.35619449,0,1.57079632679,0.785398163397])
model.forwardKinematics(q)
data = model.data
pin.updateFramePlacements(model.model, data)

 
x = data.oMf[controlled_frame].translation


print(x)