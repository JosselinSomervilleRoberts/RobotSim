import meshcat
import numpy as np


def mm_to_foot(x):
    return x * 0.00328084



class Joint:
    """General class to add a relation between 2 links"""
    def __init__(self, linkParent, linkChild, offsetTranslation = np.array([0,0,0]), offsetRotationMatrix = np.identity(4)):
        self.offsetTranslation = offsetTranslation
        self.offsetRotationMatrix = offsetRotationMatrix
        self.offsetMatrix = offsetRotationMatrix @ meshcat.transformations.translation_matrix(offsetTranslation)
        self.linkParent = linkParent
        self.linkChild = linkChild
        self.linkParent.childs.append(self)
        self.linkChild.parent = self

    def update(self):
        self.linkChild.setMatrixAndUpdate(self.linkParent.matrix @ self.offsetMatrix)



class FixedJoint(Joint):
    """Constraints 2 links together"""
    def __init__(self, linkParent, linkChild, offsetTranslation = np.array([0,0,0]), offsetRotationMatrix = np.identity(4)):
        super().__init__(linkParent, linkChild, offsetTranslation, offsetRotationMatrix)
        self.linkParent.update()

class RotationJoint(Joint):
    """Rotation relationship between 2 links"""
    def __init__(self, linkParent, linkChild, axis, offsetTranslation = np.array([0,0,0]), offsetRotationMatrix = np.identity(4), baseRotation = 0):
        super().__init__(linkParent, linkChild, offsetTranslation, offsetRotationMatrix)
        self.axis = axis
        self.baseRotation = baseRotation
        self.rotation = baseRotation + 0
        self.setRotationAndUpdate(0)
        self.linkParent.update()

    def setRotation(self, rotation):
        self.rotation = rotation + self.baseRotation

    def update(self):
        self.linkChild.setMatrixAndUpdate(self.linkParent.matrix @ self.offsetMatrix @ meshcat.transformations.rotation_matrix(self.rotation, self.axis))

    def setRotationAndUpdate(self, rotation):
        self.setRotation(rotation)
        self.update()


class TranslationJoint(Joint):
    """Translation relationship between 2 links"""
    def __init__(self, linkParent, linkChild, axis, offsetTranslation = np.array([0,0,0]), offsetRotationMatrix = np.identity(4), baseTranslation = 0):
        super().__init__(linkParent, linkChild, offsetTranslation, offsetRotationMatrix)
        if np.linalg.norm(axis) == 0: print("Non valid axis of translation, norm=0")
        self.axis = axis / np.linalg.norm(axis)
        self.baseTranslation = baseTranslation
        self.translation = baseTranslation + 0
        self.setTranslationAndUpdate(0)
        self.linkParent.update()

    def setTranslation(self, translation):
        self.translation = translation + self.baseTranslation

    def update(self):
        self.linkChild.setMatrixAndUpdate(self.linkParent.matrix @ self.offsetMatrix @ meshcat.transformations.translation_matrix(self.translation * self.axis))

    def setTranslationAndUpdate(self, translation):
        self.setTranslation(translation)
        self.update()



class Link:
    """Models a solid group of part"""
    id_counter = 0
    colors = [[1,0,0,1], [0,1,0,1],[0,0,1,1], [1,0,1,1]]

    def __init__(self, stl_path, viewer, offsetTranslation = np.array([0,0,0]), offsetRotationMatrix = np.identity(4), color = None):
        self.viewer = viewer
        self.obj = meshcat.geometry.StlMeshGeometry.from_file(stl_path)
        self.offsetTranslation = offsetTranslation
        self.offsetRotationMatrix = offsetRotationMatrix
        self.offsetMatrix = offsetRotationMatrix @ meshcat.transformations.translation_matrix(offsetTranslation)
        self.matrix = self.offsetMatrix
        self.material = meshcat.geometry.MeshLambertMaterial()
        self.id = Link.id_counter
        Link.id_counter += 1
        self.meshColor = Link.colors[self.id % len(Link.colors)]
        if color is not None: self.meshColor = color
        self.material.color = int(self.meshColor[0] * 255) * 256**2 + int(self.meshColor[1] * 255) * 256 + int(self.meshColor[2] * 255)
        self.material.reflectivity = 255
        if float(self.meshColor[3]) != 1.0:
            self.material.transparent = True
            self.material.opacity = float(self.meshColor[3])
        self.viewer[str(self.id)].set_object(self.obj, self.material)
        self.childs = []
        self.parent = None
        self.update()

    def setMatrix(self, matrix):
        self.matrix = matrix @ self.offsetMatrix

    def update(self):
        self.viewer[str(self.id)].set_transform(self.matrix)
        for joint in self.childs:
            joint.update()

    def setMatrixAndUpdate(self, matrix):
        self.setMatrix(matrix)
        self.update()


def loadLinksAndJoints(viewer, basePath, files, offsets, parents, childs, axis, jointTypes, colors = None):
    """Loads all the links defined by the files and create the joints defined by the offserts, aprents, joints, axis and jointTypes.
    Returns the tupple {links, joints"""
    links = []
    joints = []

    for i, file in enumerate(files):
        link = Link(basePath + file, viewer, color=colors[i])
        links.append(link)

    for i in range(len(offsets)):
        joint = None
        offset = mm_to_foot(np.array(offsets[i]))
        linkParent = links[parents[i]]
        linkChildren = links[childs[i]]

        jointType = str(jointTypes[i]).lower()
        if jointType == "rotation":
            joint = RotationJoint(linkParent, linkChildren, axis=axis[i], offsetTranslation=offset)
        elif jointType == "translation":
            joint = TranslationJoint(linkParent, linkChildren, axis=axis[i], offsetTranslation=offset)
        elif jointType == "fixed":
            joint = FixedJoint(linkParent, linkChildren, offsetTranslation=offset)
        else:
            print("Joint type", jointType, "nor valid.")
        if joint is not None:
            joints.append(joint)

    return links, joints