import requests
from lockfile import LockFile

import numpy as np
import scipy
import threading
import time
import matplotlib.pyplot as plt 
import json
import random
import os
from Tsolver import TsolverThread, TsolverSidecarThread


from osparc_control import CommandManifest
from osparc_control import CommandParameter
from osparc_control import CommnadType
from osparc_control import ControlInterface
from osparc_control import ControlInterface



command_generic = CommandManifest(
    action="command_generic",
    description="send some stuff",
    params=[
        CommandParameter(name="instructions", description="some instructions")
    ],
    command_type=CommnadType.WITHOUT_REPLY,
)

model_interface = ControlInterface(
    remote_host="localhost",
    exposed_interface=[command_generic],
    remote_port=1235,
    listen_port=1234,
)


# insyncpath = "test_solver/insync.json"
# outsyncpath = "test_solver/outsync.json"
# sidecarsatelite_url = "outsync.json"
# sidecar_url = "insync.json"


# if os.path.exists(insyncpath):
#       os.remove(insyncpath)
# if os.path.exists(outsyncpath):
#       os.remove(outsyncpath)


n=20; Tinit=np.zeros((n,n), float); Tsource=np.ones((n-2,n-2), float); 

thread1a=TsolverSidecarThread(model_interface)
thread2=TsolverThread(1, n, Tinit, 0.1, Tsource, 1, 1, 100, thread1a.myTSolverSideCar)
# dx, n, Tinit, dt, Tsource, k, sourcescale, tend, sidecar

# Start new Threads
model_interface.start_background_sync()

thread1a.start()
thread2.start()

threads = []
threads.append(thread2)

# Wait for all threads to complete
for t in threads:
    t.join()
thread1a.stop=True
time.sleep(0.5)
model_interface.stop_background_sync()

fig, ax = plt.subplots(1)
ax.set_aspect('equal')
T=thread2.myTsolver.T
plt.imshow(T)
plt.colorbar()
plt.savefig("tsolver_plot.png")

