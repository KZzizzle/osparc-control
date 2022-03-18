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


control_interface = ControlInterface(
    remote_host="localhost",
    exposed_interface=[command_add, command_get_time, command_print_solver_status, command_random_in_range],
    remote_port=1235,
    listen_port=1234,
)


insyncpath = "../insync.json"
outsyncpath ="../outsync.json"
sidecarsatelite_url = "outsync.json"
sidecar_url = "insync.json"


if os.path.exists(insyncpath):
      os.remove(insyncpath)
if os.path.exists(outsyncpath):
      os.remove(outsyncpath)


n=20; Tinit=np.zeros((n,n), float); Tsource=np.ones((n-2,n-2), float); 
interface = control_interface
thread1a=TsolverSidecarThread(interface)
thread2=TsolverThread(1, n, Tinit, 0.1, Tsource, 1, 1, 500, thread1a.myTSolverSideCar)
# dx, n, Tinit, dt, Tsource, k, sourcescale, tend, sidecar

# Start new Threads
control_interface.start_background_sync()

thread1a.start()
thread2.start()

threads = []
threads.append(thread2)

# Wait for all threads to complete
for t in threads:
    t.join()
thread1a.stop=True

control_interface.stop_background_sync()
