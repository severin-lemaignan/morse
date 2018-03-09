import logging; logger = logging.getLogger("morsebuilder." + __name__)
import math
from morse.core.mathutils import Vector, Matrix, Euler
import copy
import bpy

from morse.builder import bpymorse

from morse.builder.creator import ComponentCreator


URDFparser = None

try:
    from urdf_parser_py.urdf import URDF as URDFparser
    from urdf_parser_py.urdf import Mesh, Box, Cylinder, Sphere
except ImportError:
    logger.error("[URDF] to load URDF files, you must first install urdf_parser_py")


# Meshes are referenced in the URDF file relative to their package, eg:
# 'package://pepper_meshes/meshes/1.0/Torso.dae'
# MORSE will replace 'package://' by 'ROS_SHARE_ROOT':
import os
ROS_SHARE_ROOT=os.environ.get("ROS_PACKAGE_PATH","/opt/ros/kinetic/share").split(":")[0] + "/"

MATERIALS = {}

EPSILON = 0.05

class URDFLink:

    def __init__(self, urdf_link):

        self.name = urdf_link.name
        self.visual = urdf_link.visual

        # TODO: implement these
        self.inertial = urdf_link.inertial
        self.collision = urdf_link.collision

        self._get_origin()

        logger.debug("[URDF] create Link {}".format(urdf_link.name))

    def _get_origin(self):

        xyz = (0,0,0)
        rpy = (0,0,0)

        if self.visual and self.visual.origin:
            xyz = self.visual.origin.xyz
            rpy = self.visual.origin.rpy

        self.xyz = Vector(xyz)
        self.rot = Euler(rpy, 'XYZ').to_quaternion()

class URDFJoint:

    UNKNOWN = 'unknown'
    REVOLUTE = 'revolute'
    CONTINUOUS = 'continuous'
    PRISMATIC = 'prismatic'
    FLOATING = 'floating'
    PLANAR = 'planar'
    FIXED = 'fixed'

    TYPES = [UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED]

    def __init__(self, urdf_joint, urdf_link):
        
        self.name = urdf_joint.name
        self.type = urdf_joint.type

        xyz = (0,0,0)
        rpy = (0,0,0)

        if urdf_joint.origin:
            xyz = urdf_joint.origin.xyz
            if urdf_joint.origin.rpy:
                rpy = urdf_joint.origin.rpy

        self.xyz = Vector(xyz)
        self.rot = Euler(rpy, 'XYZ').to_quaternion()
        self.rot_real = Euler(rpy, 'XYZ').to_quaternion()

        self.link = URDFLink(urdf_link)

        if not urdf_joint.axis:
            # defaults to (1,0,0)
            self.axis = (1, 0, 0)
        else:
            self.axis = urdf_joint.axis

        self.limit = urdf_joint.limit

        self.children = []

        # edit/access this member *only* in EditMode
        self.editbone = None

        # edit/access this member *only* in PoseMode/ObjectMode
        self.posebone = None

        logger.debug("[URDF] create Joint {}".format(urdf_joint.name))

    def add_child(self, urdf_joint, urdf_link):

        child = URDFJoint(urdf_joint, urdf_link)
        self.children.append(child)
        return child

    def build_editmode(self, armature, parent = None):

        # we create a bone directed to y to fit
        # the world coordinate system
        self.editbone = armature.data.edit_bones.new(self.name)
        self.editbone.head = (0,0,0)
        self.editbone.tail = (0, EPSILON, 0)

        # if there is a parent, we have to set it and
        # update our rotation, because we have to use
        # the reference frame of the parent bone
        if parent:
            p_bone = parent.editbone
            self.editbone.parent = p_bone
            self.editbone.use_inherit_rotation = True
            self.rot_real = parent.rot_real * self.rot

        # apply our new rotation
        self.editbone.transform(self.rot_real.to_matrix())

        # now we set the bone position related
        # to parent (or not) and rotation
        if parent:
            self.editbone.head = p_bone.head + parent.rot_real * self.xyz
        else:
            self.editbone.head = self.xyz

        self.editbone.tail = self.editbone.head + self.rot_real * Vector((0, EPSILON, 0))

        # do this procedure for all child joints (bones)
        for child in self.children:
            child.build_editmode(armature, self)

    def build_objectmode(self, armature, parent = None):

        try:
            self.posebone = armature.pose.bones[self.name]
        except KeyError:
            logger.error("[ERROR][URDF] bone %s not yet added to the armature" % self.name)
            return

        self.configure_joint(self.posebone)

        self.add_link_frame(armature)

        for child in self.children:
            child.build_objectmode(armature, self)

    def configure_joint(self, posebone):

        posebone.lock_location = (True, True, True)
        posebone.lock_rotation = (True, True, True)
        posebone.lock_scale = (True, True, True)

        ax = self.axis[0]
        ay = self.axis[1]
        az = self.axis[2]

        if self.limit:
            lu = self.limit.upper
            ll = self.limit.lower

            # if we have negativ axis like axis="0 -1 0"
            # we have to reverse the limits and negate them
            limit_x_lower = ll if (ax > 0) else -lu
            limit_x_upper = lu if (ax > 0) else -ll

            limit_y_lower = ll if (ay > 0) else -lu
            limit_y_upper = lu if (ay > 0) else -ll

            limit_z_lower = ll if (az > 0) else -lu
            limit_z_upper = lu if (az > 0) else -ll

        if self.type == self.FIXED:
            # This is not really a joint because it cannot move.
            # All degrees of freedom are locked.
            return

        elif self.type == self.REVOLUTE:
            # a hinge joint that rotates along the axis and has
            # a limited range specified by the upper and lower limits. 

            if self.limit:
                c = posebone.constraints.new('LIMIT_ROTATION')
                c.owner_space = 'LOCAL'

            if ax: # x-axis
                posebone.lock_rotation[0] = False

                if self.limit:
                    c.use_limit_x = True
                    c.min_x = limit_x_lower
                    c.max_x = limit_x_upper

            if ay: # y-axis
                posebone.lock_rotation[1] = False

                if self.limit:
                    c.use_limit_y = True
                    c.min_y = limit_y_lower
                    c.max_y = limit_y_upper

            if az: # z-axis
                posebone.lock_rotation[2] = False

                if self.limit:
                    c.use_limit_z = True
                    c.min_z = limit_z_lower
                    c.max_z = limit_z_upper


        elif self.type == self.PRISMATIC:
            # a sliding joint that slides along the axis, and has a
            # limited range specified by the upper and lower limits

            if self.limit:
                c = posebone.constraints.new('LIMIT_LOCATION')
                c.owner_space = 'LOCAL'

            if ax: # x-axis
                posebone.lock_location[0] = False

                if self.limit:
                    c.use_min_x = True
                    c.use_max_x = True
                    c.min_x = limit_x_lower
                    c.max_x = limit_x_upper

            if ay: # y-axis
                posebone.lock_location[1] = False

                if self.limit:
                    c.use_min_y = True
                    c.use_max_y = True
                    c.min_y = limit_y_lower
                    c.max_y = limit_y_upper

            if az: # z-axis
                posebone.lock_location[2] = False

                if self.limit:
                    c.use_min_z = True
                    c.use_max_z = True
                    c.min_z = limit_z_lower
                    c.max_z = limit_z_upper

        elif self.type == self.CONTINUOUS:
            # a continuous hinge joint that rotates around
            # the axis and has no upper and lower limits 

            if ax: # x-axis
                posebone.lock_rotation[0] = False

            if ay: # y-axis
                posebone.lock_rotation[1] = False

            if az: # z-axis
                posebone.lock_rotation[2] = False

        else:
            logger.warning("[URDF] joint type ({}) configuration not implemented yet".format(self.type))

    def add_link_frame(self, armature):
        """ Creates the link-visual geometry, adds the material
            and relate it to the parent bone.
        """

        if not self.link.visual:
            logger.debug("[URDF] no visual found for {}".format(self.link.name))
            return

        visuals = create_objects_by_link(self.link)

        for v in visuals:
            self.add_material(v)

            # parent the visuals to the armature
            v.parent = armature
            v.parent_bone = self.name
            v.parent_type = "BONE"

            # visual has to be attached on bone's head
            vector = Vector([abs(k) for k in self.rot_real * self.posebone.vector])
            v.location -= vector

    def add_material(self, obj):
        """ Adding material to scene if not exist and let
            the object use it. We differentiate between local
            and global material (local is first priority).
            We ignore the alpha value of the material color.
            TODO: Adding Texture
        """

        # cannot add materials to types like CAMERA etc.
        if obj.type != 'MESH':
            return

        if not self.link.visual or not self.link.visual.material:
            return

        material = self.link.visual.material

        if not material.name:
            logger.warning("[URDF] found material without name: {}".format(self.link.material))
            return

        rgba = None
        texture = None

        # local material
        if material.color or material.texture:
            if material.color:
                rgba = material.color.rgba

            if material.texture:
                texture = material.texture

        # global material
        else:
            if material.name not in MATERIALS:
                logger.warning("[URDF] global material not found: {}".format(material.name))
                return

            rgba = MATERIALS[material.name]['color']
            texture = MATERIALS[material.name]['texture']


        mat = bpymorse.get_material(material.name)
        if not mat:
            mat = bpymorse.get_materials().new(name=material.name)

        obj.data.materials.append(mat)
        mat.diffuse_color = (rgba[0], rgba[1], rgba[2])

    def __repr__(self):
        return "URDF joint<%s>" % self.name

class URDF:

    def __init__(self, urdf):

        self.urdf_file = urdf
        if URDFparser is None:
            logger.error("[URDF] Can not load URDF file: urdf_parser_py not available")
            return

        self.urdf = URDFparser.from_xml_string(open(urdf,'r').read())

        self.name = self.urdf.name

        for mat in self.urdf.materials:
            add_material(mat)

        self.base_link = URDFLink(self.urdf.link_map[self.urdf.get_root()])
        self.roots = self._walk_urdf(self.urdf.link_map[self.urdf.get_root()])

    def _walk_urdf(self, link, parent_bone = None):

        bones = []
        for joint, child_link in self._get_urdf_connections(link):
            if parent_bone:
                bone = parent_bone.add_child(joint, child_link)
            else:
                bone = URDFJoint(joint, child_link)

            self._walk_urdf(child_link, bone)
            bones.append(bone)
        return bones

    def _get_urdf_connections(self, link):

        joints = [joint for joint in self.urdf.joints if joint.parent == link.name]
        return [(joint, self.urdf.link_map[joint.child]) for joint in joints]


    def build(self):
        """
        Create the armature and linked objects from the URDF description given at
        class instantiation.

        :return: the root object created from the URDF file
        """

        if self.urdf is None:
            return None

        bpymorse.add_object(
            type='ARMATURE', 
            enter_editmode=True,
            location=(0,0,0))
        ob = bpymorse.get_context_object()
        ob.show_x_ray = True
        ob.name = self.name
        amt = ob.data
        amt.name = self.name+'_armature'
        amt.show_axes = True

        bpymorse.mode_set(mode='EDIT')
        for root in self.roots:
            root.build_editmode(ob)

        # creating base-link visual, if exist
        if self.base_link.visual:
            visuals = create_objects_by_link(self.base_link)
            for v in visuals:
                v.parent = ob
                v.parent_type = "OBJECT"

        bpymorse.mode_set(mode='OBJECT')

        for root in self.roots:
            root.build_objectmode(ob)

        return ob

def add_material(urdf_material):
    """ Add a urdf_material to the global MATERIALS dictonary.
    """

    global MATERIALS

    if urdf_material.name in MATERIALS:
        return

    MATERIALS[urdf_material.name] = {'color': None, 'texture': None}

    if urdf_material.color and urdf_material.color.rgba:
        MATERIALS[urdf_material.name]['color'] = urdf_material.color.rgba

    if urdf_material.texture and urdf_material.texture.filename:
        MATERIALS[urdf_material.name]['texture'] = urdf_material.texture.filename

def create_objects_by_link(link):
    """ Creates a object from a given URDFLink and
        set the correct origin.

        :param link: related URDFLink
        :return: list of all created objects
    """

    visuals = []

    if link.visual and link.visual.geometry:
        geometry = link.visual.geometry

        if isinstance(geometry, Mesh):

            path = geometry.filename.replace("package:/", ROS_SHARE_ROOT)
            path = path.replace("file://", "")

            # Save a list of objects names before importing Collada/STL
            objects_names = [obj.name for obj in bpymorse.get_objects()]

            if '.dae' in geometry.filename:
                bpymorse.collada_import(filepath=path)

            elif '.stl' in geometry.filename:
                bpymorse.stl_import(filepath=path)

            # Get a list of the imported objects
            visuals = [obj for obj in bpymorse.get_objects() \
                          if obj.name not in objects_names]

            if geometry.scale:
                for v in visuals:
                    v.scale = [v.scale[0] * geometry.scale[0],
                               v.scale[1] * geometry.scale[1],
                               v.scale[2] * geometry.scale[2]]

        elif isinstance(geometry, Box):
            bpymorse.add_mesh_cube()
            ob = bpymorse.active_object()
            ob.name = link.name
            ob.dimensions = geometry.size
            visuals = [ob]

        elif isinstance(geometry, Cylinder):
            radius = geometry.radius
            length = geometry.length
            bpymorse.add_mesh_cylinder(radius=radius, depth=length)
            ob = bpymorse.active_object()
            ob.name = link.name
            visuals = [ob]

        elif isinstance(geometry, Sphere):
            bpymorse.add_mesh_uv_sphere(size=geometry.radius)
            ob = bpymorse.active_object()
            ob.name = link.name
            visuals = [ob]

    else:
        bpymorse.add_empty(type = "ARROWS")

        empty = bpymorse.get_first_selected_object()
        empty.name = link.name
        empty.scale = [0.01, 0.01, 0.01]
        visuals = [empty]

    for v in visuals:
        # set link origin and rotation
        v.location = link.xyz
        v.rotation_mode = "QUATERNION"
        v.rotation_quaternion = link.rot
        bpymorse.origin_set(type='ORIGIN_CURSOR')

    # currently we ignore cameras and lamps
    # which could be imported by collada/stl files
    del_visuals = [v for v in visuals if v.type in ('CAMERA', 'LAMP')]
    visuals = [v for v in visuals if v.type not in ('CAMERA', 'LAMP')]

    if del_visuals:
        bpymorse.delete(del_visuals)

    return visuals
