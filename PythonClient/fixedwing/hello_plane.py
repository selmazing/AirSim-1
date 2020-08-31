import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint

# connect to AirSim simulator
client = airsim.FixedWingClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getFixedWingState()
s = pprint.pformat(state)
print("state: %s" % s)

client.armDisarm(False)
client.reset()

client.enableApiControl(False)
