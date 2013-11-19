import sys

from pyengineapi import PyEngineAPI

import morse.core
morse.core.api = PyEngineAPI

from morse.blender import main


#import pdb;pdb.set_trace()

main.init(None)

