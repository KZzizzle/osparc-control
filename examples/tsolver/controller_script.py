from lockfile import LockFile
import requests

import numpy as np
import time
import matplotlib.pyplot as plt 
from Controller import ControllerThread, TsolverSidecarSateliteThread


from osparc_control import CommandManifest
from osparc_control import CommandParameter
from osparc_control import CommnadType
from osparc_control import ControlInterface


command_data = CommandManifest(
    action="command_data",
    description="receive some stuff",
    params=[
        CommandParameter(name='t', description="time"),
        CommandParameter(name='endsignal', description="end?"),
        CommandParameter(name='paused', description="is paused"), 
        CommandParameter(name='records', description="some records")
    ],
    command_type=CommnadType.WITHOUT_REPLY,
)

control_interface = ControlInterface(
    remote_host="localhost",
    exposed_interface=[command_data],
    remote_port=1234,
    listen_port=1235,
)


n=20; Tinit=np.zeros((n,n), float); Tsource=np.ones((n-2,n-2), float); 


thread1b=TsolverSidecarSateliteThread(control_interface)
thread3=ControllerThread('sourcescale', 1, 'Tpoint', [10,10], 4, 5, 0.01, 0, 0, thread1b.myTSolverSideCarSatelite)
#tweakparam_key, regulationparam_key, regulationparam_otherparams, setpoint, iteration_time, KP, KI, KD

# Start threads
control_interface.start_background_sync()

thread3.start()

threads = []
threads.append(thread3)

# Wait for all threads to complete
for t in threads:
    t.join()

time.sleep(0.5)
thread1b.stop=True
time.sleep(0.5)
control_interface.stop_background_sync()


fig, (ax1, ax2,ax3) = plt.subplots(3)
ax1.plot(thread3.myController.sets)
ax1.set_title('set value')
ax2.plot(thread3.myController.errors)
ax2.set_title('errors')
ax3.plot(thread3.myController.controlledvals)
ax3.set_title('controlled value')

plt.savefig("controller_plot.png")
print("Exiting Main Thread")
