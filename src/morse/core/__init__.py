# running in Blender?
try:
    from morse.core.api_blender import BlenderAPI
    blenderapi = BlenderAPI()
except ImportError:
    from morse.core.engineapi import EngineAPI
    # Can fail if we are in Blender but not yet in the GameEngine,
    # typically at 'Builder' stage.
    blenderapi = EngineAPI()
