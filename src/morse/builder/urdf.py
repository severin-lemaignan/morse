import bpy, math
from mathutils import Vector, Matrix, Euler

#from morse.builder import bpymorse

from urdf_parser_py.urdf import URDF

URDFMODEL="/home/lemaigna/my_ros_prefix/share/nao_description/urdf/nao_robot_v4.urdf"

robot = URDF.from_xml_string(open(URDFMODEL,'r').read())

class URDFLink:
    def add_empty(self):
        return
        assert(parent)
        bone = None

        # fixed joint at the end of the kinematic chain: we create an empty

        bpy.ops.object.mode_set(mode='OBJECT')

        #bpymorse.add_morse_empty("ARROWS")
        bpy.ops.object.empty_add(type = "ARROWS")

        #empty = bpymorse.get_first_selected_object()
        empty = bpy.context.selected_objects[0]

        empty.name = self.name
        empty.matrix_local = armature.data.bones[parent_name].matrix_local
        empty.location = armature.data.bones[parent_name].tail_local

        # TODO: parenting -> select empty, armature, correct bone, and then bpy.ops.object.parent_set(type='BONE')
        #bpymorse.select_only(armature)
        bpy.ops.object.select_all(action='DESELECT')
        armature.select = True

        bpy.ops.object.mode_set(mode='EDIT')

class URDFJoint:

    # cf urdf_parser_py.urdf.Joint.TYPES
    FIXED = "fixed"
    PRISMATIC = "prismatic"
    REVOLUTE = "revolute"

    TYPES = [FIXED, PRISMATIC, REVOLUTE]

    def __init__(self, urdf_joint):
        
        self.name = urdf_joint.name
        self.type = urdf_joint.type
        self.xyz = urdf_joint.origin.xyz
        if urdf_joint.origin.rpy:
            self.rot = Euler(urdf_joint.origin.rpy, 'XYZ').to_quaternion()
        else:
            self.rot = Euler((0,0,0)).to_quaternion()

        self.children = []

        self.bone = None

    def add_child(self, urdf_joint):

        child = URDFJoint(urdf_joint)
        
        if child.xyz == [0.,0.,0.]:
            print("Zero lenght bones (ie, multi DoF joints) are not yet fully supported.")
            self.name += ">%s" % urdf_joint.name
            self.rot = self.rot * child.rot #TODO -> does not work yet...
            return self
        else:
            child = URDFJoint(urdf_joint)
            self.children.append(child)
            return child

    def build(self, armature, parent = None):
        # Create bones, must be called in EDIT mode!

        # Forget the crappy statement that states you should forget the crap below
        #
        # URDF joints map to bones, URDF links map to objects whose origin is an
        # empty located at the joint's bone's HEAD.
        #
        # Forget the crap below.
        # URDF links must map to bones. Links connected by joints whose origin is
        # (0,0,0) must be combined into a single bone
        # The naming of joints needs to be stored somewhere else.
        # actually, one Blender bone = joint + child link
        # one link may be connected to several other (child) links at different
        # places -> the tail of the parent *does not have* to be connected to
        # the head of the child. The head of the child *is* the joint origin
        # The remaining question are: 
        #   - where goes the tail of a bone?
        #   - management of multi-DoF

        # The URDF joint's origin (ie, the pose of the bone HEAD) is defined
        # relative to the parent joint (ie, the parent's bone HEAD). Hence, the
        # *parent*'s bone TAIL must be placed at the current joint origin.
        #
        # As a consequence, when creating the bone, we have no clue regarding
        # the position of the tail: we need a child joint for that.
        #
        # If the joints at the end of the kinematic chains are *fixed* joints,
        # we are safe: we can set the previous joint's bone's tail, and we
        # simply create an empty instead of a bone for the current (fixed)
        # joint.  If the last joint is not fixed, we need to create a bone, and
        # place the tail of this bone orthogonal to the rotation axis (for
        # revolute joints) or along the sliding axis (for prismatic joints).

        print("Building %s..." % self.name)
        self.bone = armature.data.edit_bones.new(self.name)

        if parent:
            self.bone.use_inherit_rotation = True
            self.bone.parent = parent.bone

            if len(parent.children) == 1:

                # this joint is the only child of the parent joint: connect them
                # place parent's tail
                parent.bone.tail = self.rot * Vector(self.xyz) + parent.bone.head
                self.bone.use_connect = True
            else:
                self.bone.head = self.rot * Vector(self.xyz) + parent.bone.head

        else:
            self.bone.head = (0,0,0)

        # TODO: tail pose should default to a position orthogonal to the rotation axis
        self.bone.tail = self.bone.head + Vector((0,0,0.1))

        for child in self.children:
            child.build(armature, self)

    def __repr__(self):
        return "URDF joint<%s>" % self.name

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
                bone = URDFJoint(joint)

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
        amt.name = self.name+'_armature'
        amt.show_axes = True

        bpy.ops.object.mode_set(mode='EDIT')
        for root in self.roots:
            root.build(ob)
        bpy.ops.object.mode_set(mode='OBJECT')

armature = URDFArmature("nao", robot)
armature.build()
