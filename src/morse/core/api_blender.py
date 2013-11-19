""" This module wraps the calls to the Blender Python API. This is intended
for all the cases we need to run MORSE code outside Blender (mostly for
documentation generation purposes).
"""
import logging; logger = logging.getLogger("morse." + __name__)

import sys
import bpy
import bge

from morse.core.engineapi import EngineAPI, PersistantStorage

class BlenderAPI(EngineAPI):

    UPARROWKEY = bge.events.UPARROWKEY
    DOWNARROWKEY = bge.events.DOWNARROWKEY
    RIGHTARROWKEY = bge.events.RIGHTARROWKEY
    LEFTARROWKEY = bge.events.LEFTARROWKEY
    LEFTCTRLKEY = bge.events.LEFTCTRLKEY
    LEFTALTKEY = bge.events.LEFTALTKEY
    AKEY = bge.events.AKEY
    BKEY = bge.events.AKEY
    DKEY = bge.events.DKEY
    EKEY = bge.events.EKEY
    FKEY = bge.events.FKEY
    IKEY = bge.events.IKEY
    JKEY = bge.events.JKEY
    KKEY = bge.events.KKEY
    LKEY = bge.events.LKEY
    NKEY = bge.events.NKEY
    OKEY = bge.events.OKEY
    QKEY = bge.events.QKEY
    RKEY = bge.events.RKEY
    SKEY = bge.events.SKEY
    UKEY = bge.events.UKEY
    VKEY = bge.events.UKEY
    WKEY = bge.events.WKEY
    XKEY = bge.events.XKEY
    ZKEY = bge.events.ZKEY

    LEFTMOUSE = bge.events.LEFTMOUSE
    RIGHTMOUSE = bge.events.RIGHTMOUSE

    F8KEY = bge.events.F8KEY
    F5KEY = bge.events.F5KEY

    fake = False

    def __init__(self):
        logger.info("Initializing MORSE interface with Blender.")

    @staticmethod
    def input_active():
        return bge.logic.KX_INPUT_ACTIVE

    @staticmethod
    def input_just_activated():
        return bge.logic.KX_INPUT_JUST_ACTIVATED

    @staticmethod
    def input_just_released():
        return bge.logic.KX_INPUT_JUST_RELEASED

    @staticmethod
    def input_none():
        return bge.logic.KX_INPUT_NONE


    @staticmethod
    def controller():
        return bge.logic.getCurrentController()

    @staticmethod
    def scene():
        return bge.logic.getCurrentScene()

    @staticmethod
    def add_scene(name, overlay=1):
        return bge.logic.addScene(name, overlay)

    @staticmethod
    def get_scene_list():
        return bge.logic.getSceneList()

    @staticmethod
    def get_scene_map():
        return {s.name: s for s in bge.logic.getSceneList()}

    @staticmethod
    def render():
        return bge.render


    @staticmethod
    def hascameras():
        return hasattr(bge.logic, 'cameras')


    @staticmethod
    def initcameras():
        bge.logic.cameras = {}


    @staticmethod
    def cameras():
        return bge.logic.cameras


    @staticmethod
    def mousepointer(visible = True):
        bge.logic.mouse.visible = visible


    @staticmethod
    def constraints():
        return bge.constraints


    @staticmethod
    def texture():
        return bge.texture

    @staticmethod
    def objectdata(name):
        return bpy.data.objects[name]

    @staticmethod
    def materialdata(name):
        return bpy.data.materials[name]


    @staticmethod
    def getalwayssensors(obj):
        return [s for s in obj.sensors if isinstance(s, bge.types.SCA_AlwaysSensor)]

    @staticmethod
    def getfrequency():
        return bge.logic.getLogicTicRate()

    def persistantstorage(self):
        """Simply returns the bge.logic object, that persists
        between calls to the script.
        """
        if not hasattr(bge.logic, "morsedata"):
            bge.logic.morsedata = PersistantStorage()
        return bge.logic.morsedata

    @staticmethod
    def version():
        return bpy.app.version

    @staticmethod
    def getssr():
        return bge.logic.getCurrentScene().objects["Scene_Script_Holder"]

    @staticmethod
    def joysticks():
        return bge.logic.joysticks

    @staticmethod
    def isfastmode():
        for area in bpy.context.window.screen.areas:
            if area.type == 'VIEW_3D':
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        return space.viewport_shade == 'WIREFRAME'
