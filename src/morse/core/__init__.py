import logging; logger = logging.getLogger("morse." + __name__)

# did some forced the API to use?
if 'api' in globals():
    logger.warn("Force loading of API <%s>" % api.__name__)
    blenderapi = api()
else: # try to auto-detect
    try:
        # running in Blender?
        from morse.core.api_blender import BlenderAPI
        blenderapi = BlenderAPI()
    except ImportError:
        from morse.core.engineapi import EngineAPI
        # Can fail if we are in Blender but not yet in the GameEngine,
        # typically at 'Builder' stage.
        blenderapi = EngineAPI()
