import math
import time

import meshcat
import numpy as np
from viewer import loadLinksAndJoints

viewer = meshcat.Visualizer()
vis = viewer.open()


# Links
files = ["base_assembled.STL", "Link1_assembled.STL", "Link2_assembled_without_rack.STL", "rack.STL", "gripper_link_1.STL", "gripper_link_2.STL", "gripper_link_3_1.STL", "gripper_link_3_2.STL"]
colors = [[0.5,0.5,0.5,1], [0.3,0.3,0.3,1], [0.5,0.5,0.5,1], [0.2,0.2,0.2,1], [0.2,0.2,0.2,1], [0.4,0.4,0.4,1], [0.3,0.2,0.15,1], [0.3,0.2,0.15,1]]

# Joints
offsets = [[0, 0, 479.42], [447.76, 0, 33.4], [293.37, -20.33, -8.04], [6.35, 0, -303], [24.7, 0, -85.77], [19.5, 15.52-36, 39], [19.5, 15.52, 39]]
parents = [0, 1, 2, 3, 4, 5, 5]
childs =  [1, 2, 3, 4, 5, 6, 7]
axis = [[0,0,1], [0,0,1], [0,0,1], None, [1,0,0], [0,0,1], [0,0,1]]
jointTypes = ["rotation", "rotation", "translation", "fixed", "rotation", "rotation", "rotation"]

links, joints = loadLinksAndJoints(viewer, "CAD/", files, offsets, parents, childs, axis, jointTypes, colors)
#joint = RotationJoint(links[-3], links[-1], axis=axis[-1], offsetTranslation=mm_to_foot(np.array(offsets[-1])))


for i in range(2000):
    theta = (i + 1) / 1000 * 2 * math.pi
    theta2 = 0.5 * math.pi * np.sin(i/500 * 2 * math.pi)**2
    t = 0.2 * (1 - np.cos(i/500 * 2 * math.pi))
    now = time.time()
    A0 = np.array([0, 0, 1])
    joints[1].setRotation(-2*theta2)
    joints[4].setRotation(theta)
    joints[-2].setRotation(-theta2)
    joints[-1].setRotation(theta2)
    joints[2].setTranslation(t)
    joints[0].setRotationAndUpdate(theta2)
    #vis["0"].set_transform(meshcat.transformations.rotation_matrix(theta, A0) @ meshcat.transformations.translation_matrix([1,0,0]))
    time.sleep(0.02)