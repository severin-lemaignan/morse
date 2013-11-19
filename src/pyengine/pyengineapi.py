import logging; logger = logging.getLogger("morse." + __name__)

import sys

from morse.core.engineapi import EngineAPI, PersistantStorage

VERSION = (0,1,0)

class ActuatorManager:
    def __init__(self):
        self.actuators = {"Quit_sim": None}

    def activate(self, actuator):
        pass

class SceneManager:
    objects = []

class PyEngineAPI(EngineAPI):

    fake = False

    def __init__(self):
        super().__init__()
        logger.info("Initializing MORSE interface with pyEngine.")
        self._scene = SceneManager()
        self._actuators = ActuatorManager()

    @staticmethod
    def input_active():
        raise NotImplementedError

    @staticmethod
    def input_just_activated():
        raise NotImplementedError

    @staticmethod
    def input_just_released():
        raise NotImplementedError

    @staticmethod
    def input_none():
        raise NotImplementedError

    def controller(self):
        return self._actuators

    def scene(self):
        return self._scene

    @staticmethod
    def add_scene(name, overlay=1):
        raise NotImplementedError

    @staticmethod
    def get_scene_list():
        raise NotImplementedError

    @staticmethod
    def get_scene_map():
        raise NotImplementedError

    @staticmethod
    def render():
        raise NotImplementedError

    @staticmethod
    def hascameras():
        raise NotImplementedError

    @staticmethod
    def initcameras():
        raise NotImplementedError

    @staticmethod
    def cameras():
        raise NotImplementedError

    @staticmethod
    def mousepointer(visible = True):
        raise NotImplementedError

    @staticmethod
    def constraints():
        raise NotImplementedError

    @staticmethod
    def texture():
        raise NotImplementedError

    @staticmethod
    def objectdata(name):
        raise NotImplementedError

    @staticmethod
    def materialdata(name):
        raise NotImplementedError

    @staticmethod
    def getalwayssensors(obj):
        raise NotImplementedError

    @staticmethod
    def getfrequency():
        return 0

    @staticmethod
    def version():
        return VERSION

    @staticmethod
    def getssr():
        raise NotImplementedError

    @staticmethod
    def joysticks():
        raise NotImplementedError

    @staticmethod
    def isfastmode():
        return True






