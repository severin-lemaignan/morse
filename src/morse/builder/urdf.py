import bpy, math
from mathutils import Vector, Matrix, Euler

from urdf_parser_py.urdf import URDF

URDFMODEL="/home/lemaigna/my_ros_prefix/share/nao_description/urdf/nao_robot_v4.urdf"

robot = URDF.from_xml_string(open(URDFMODEL,'r').read())

class URDFBone:

    def __init__(self, urdf_joint):
        
        self.name = urdf_joint.name
        self.xyz = urdf_joint.origin.xyz
        if urdf_joint.origin.rpy:
            self.rot = Euler(urdf_joint.origin.rpy, 'XYZ').to_quaternion()
        else:
            self.rot = Euler((0,0,0)).to_quaternion()

        self.children = []

    def add_child(self, urdf_joint):

        bone = URDFBone(urdf_joint)
        
        if bone.xyz == [0.,0.,0.]:
            print("Zero lenght bones (ie, multi DoF joints) are not yet fully supported.")
            self.name += ">%s" % urdf_joint.name
            self.rot = self.rot * bone.rot #TODO -> does not work yet...
            return self
        else:
            bone = URDFBone(urdf_joint)
            self.children.append(bone)
            return bone

    def build(self, armature, parent = None):
        # Create bones, must be called in EDIT mode!
        bone = armature.edit_bones.new(self.name)
        if parent:
            bone.use_inherit_rotation = True
            bone.parent = parent
            bone.use_connect = True
        else:
            bone.head = (0,0,0)

        bone.tail = self.rot * Vector(self.xyz) + bone.head

        for child in self.children:
            child.build(armature, bone)



class URDFArmature:

    def __init__(self, name, urdf):

        self.name = name
        self.urdf = urdf

        self.roots = self._walk_urdf(self.urdf.link_map[urdf.get_root()])

    def _walk_urdf(self, link, parent_bone = None):
        bones = []
        for joint, child_link in self._get_urdf_connections(link):
            if parent_bone:
                # be careful: if joint is a zero-length joint,
                # it gets merged into parent bone (because Blender does not
                # support 0-length joints)! so bone and parent_bone
                # may be equal.
                bone = parent_bone.add_child(joint)
            else:
                bone = URDFBone(joint)

            self._walk_urdf(child_link, bone)
            bones.append(bone)
        return bones

    def _get_urdf_connections(self, link):
        joints = [joint for joint in self.urdf.joints if joint.parent == link.name]
        return [(joint, self.urdf.link_map[joint.child]) for joint in joints]


    def build(self):
        # Create armature and object
        bpy.ops.object.add(
            type='ARMATURE', 
            enter_editmode=True,
            location=(0,0,0))
        ob = bpy.context.object
        ob.show_x_ray = True
        ob.name = self.name
        amt = ob.data
        amt.name = self.name+'Armature'
        amt.show_axes = True

        bpy.ops.object.mode_set(mode='EDIT')
        for root in self.roots:
            root.build(amt)
        bpy.ops.object.mode_set(mode='OBJECT')

armature = URDFArmature("nao", robot)
armature.build()