import requests
from lockfile import LockFile

import numpy as np
import time
import matplotlib.pyplot as plt 
from EMSolver import EMsolverThread, EMsolverSidecarThread


from osparc_control import CommandManifest
from osparc_control import CommandParameter
from osparc_control import CommnadType
from osparc_control import ControlInterface



command_instruct = CommandManifest(
    action="command_instruct",
    description="Execution Instructions",
    params=[
        CommandParameter(name="instructions", description="Instructions for execution.")
    ],
    command_type=CommnadType.WITHOUT_REPLY,
)

command_retrieve = CommandManifest(
    action="command_retrieve",
    description="gets state",
    params=[],
    command_type=CommnadType.WITHOUT_REPLY,
)

model_interface = ControlInterface(
    remote_host="localhost",
    exposed_interface=[command_instruct, command_retrieve],
    remote_port=1237,
    listen_port=1234,
)


n=20; 
Tinit=np.zeros((n,n), float); 
# Tsource=np.ones((n-2,n-2), float); 
EMsource=np.zeros((n,n), float)
EMsource[int(n/3),int(n/4)]=1
EMinit=np.zeros((n,n), float)

# threadT1a=TsolverSidecarThread(locksT,syncpathsT)
# threadT1b=TsolverSidecarSateliteThread(locksT,syncpathsT)
threadEM1a=EMsolverSidecarThread(model_interface)
# threadEM1b=EMsolverSidecarSateliteThread(model_interface)
# threadT2=TsolverThread(1, n, Tinit, 0.1, Tsource, 1, 10, 1, 5, threadT1a.myTSolverSideCar)

threadEM2=EMsolverThread(n, EMinit, 1, EMsource, Tinit, 5, 1, 5, threadEM1a.myEMSolverSideCar)
# thread3Coupling=EM_T_couplerThread(threadEM1b.myEMSolverSideCarSatelite,threadT1b.myTSolverSideCarSatelite,'T','SARvol',[1,1,n-1,n-1],'SARsource','Tvol',[0,0,n,n],1)


# Start new Threads
model_interface.start_background_sync()

# threadT1a.start()
# threadT1b.start()
threadEM1a.start()
# threadEM1b.start()
# threadT2.start()
threadEM2.start()
# thread3Coupling.start()

threads = []
# threads.append(threadT2)
threads.append(threadEM2)
# threads.append(thread3Coupling)

# Wait for all threads to complete
for t in threads:
    t.join()
# threadT1a.stop=True
# threadT1b.stop=True
threadEM1a.stop=True
# threadEM1b.stop=True

time.sleep(0.5)

model_interface.stop_background_sync()

# plt.figure(figsize=(15,5))
# plt.subplot(1, len(thread3Coupling.myCoupler.Tstored), 1)
# print('Temperature evolution')
# for i in range(len(thread3Coupling.myCoupler.Tstored)):
#     plt.subplot(1, 5, i+1)
#     plt.imshow(thread3Coupling.myCoupler.Tstored[i], vmin=0, vmax=1.3)
# plt.show()
# plt.figure(figsize=(15,5))
# plt.subplot(1, len(thread3Coupling.myCoupler.Tstored), 1)
# print('EM evolution')
# for i in range(len(thread3Coupling.myCoupler.EMstored)):
#     plt.subplot(1, 5, i+1)
#     plt.imshow(thread3Coupling.myCoupler.EMstored[i], vmin=0, vmax=1.5)
# plt.show()



fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.set_aspect('equal')
plt.imshow(threadEM2.myEMsolver.SAR)
plt.colorbar()
plt.savefig("emsolver_plot.png")