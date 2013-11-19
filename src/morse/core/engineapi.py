import logging; logger = logging.getLogger("morse." + __name__)

class PersistantStorage(dict):
        __getattr__= dict.__getitem__
        __setattr__= dict.__setitem__
        __delattr__= dict.__delitem__


class EngineAPI:

    UPARROWKEY = None
    DOWNARROWKEY = None
    RIGHTARROWKEY = None
    LEFTARROWKEY = None
    LEFTCTRLKEY = None
    LEFTALTKEY = None

    AKEY = None
    BKEY = None
    DKEY = None
    EKEY = None
    FKEY = None
    IKEY = None
    JKEY = None
    KKEY = None
    LKEY = None
    NKEY = None
    OKEY = None
    QKEY = None
    RKEY = None
    SKEY = None
    UKEY = None
    VKEY = None
    WKEY = None
    XKEY = None
    ZKEY = None

    LEFTMOUSE = None
    RIGHTMOUSE = None

    F5KEY = None
    F8KEY = None

    fake = True

    def __init__(self):
        logger.debug("Loading fake interface to simulation engine.")
        self._persistantstorage = PersistantStorage()

    @staticmethod
    def input_active():
        return None

    @staticmethod
    def input_just_activated():
        return None

    @staticmethod
    def input_just_released():
        return None

    @staticmethod
    def input_none():
        return None

    @staticmethod
    def controller():
        return None

    @staticmethod
    def scene():
        return None

    @staticmethod
    def add_scene(name, overlay=1):
        return None

    @staticmethod
    def get_scene_list():
        return None

    @staticmethod
    def get_scene_map():
        return None

    @staticmethod
    def render():
        return None

    @staticmethod
    def hascameras():
        return None

    @staticmethod
    def initcameras():
        pass

    @staticmethod
    def cameras():
        return None

    @staticmethod
    def mousepointer(visible = True):
        pass

    @staticmethod
    def constraints():
        return None

    @staticmethod
    def texture():
        return None

    @staticmethod
    def objectdata(name):
        return None

    @staticmethod
    def materialdata(name):
        return None

    @staticmethod
    def getalwayssensors(obj):
        return []

    @staticmethod
    def getfrequency():
        return 0

    def persistantstorage(self):
        """Simply returns the bge.logic object, that persists
        between calls to the script.
        """
        return self._persistantstorage

    @staticmethod
    def version():
        return 0,0,0

    @staticmethod
    def getssr():
        return None

    @staticmethod
    def joysticks():
        return None

    @staticmethod
    def isfastmode():
        return True



