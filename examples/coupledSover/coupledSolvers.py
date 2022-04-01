#!/usr/bin/env python
# coding: utf-8

# ## SIMCORE cheat sheet
# 
# ### Kernel agnostic access to the node input/output ports
# 
# Input data is automatically downloaded to __~/inputs/__
# - port of type file is available in: __~/inputs/MyPortKey/__
#   In case of compressed file (zip or tgz) the uncompressed files are available in the folder
# - port of any other type (number, integer, string) is available in: __~/inputs/key_values.json__  
# 
# Output data is automatically uploaded from __~/outputs/__
# - port of type file shall be saved to: __~/outputs/MyPortKey/__
#   In case of multiple files, they will be automatically compressed and uploaded.
# - port of any other type (number, integer, string) shall be saved in: __~/outputs/key_values.json__
#   ```
#   #key_values.json
#   {
#       "myIntValue": 5,
#       "myNumberValue": 5.0,
#       "myStringValue": "Hello world"
#   }
#   ```
# 
# ### python3  - Direct access to the node input/output ports using simcore-sdk
# 
# ```python
# from simcore_sdk import node_ports
# PORTS = await node_ports.ports() # returns a Nodeports instance with all the input/output data
# ```
# Importing from input port 0
# ```python
# #downloads the file from the storage to the notebook and returns its path
# input_1 = await (await PORTS.inputs)[0].get() 
# #read the text inside the file (only if it's a text file)
# text = input_1.read_text()
# ```
# Exporting to output port 0
# ```python
# # create a dummy file
# dummy_file_path = Path("dummy_file.csv")
# # write some text in the file
# dummy_file_path.write_text("Hello from notebook!!")
# # set this file as being the output 0 of the node (internally uploads the file to the storage)
# await (await PORTS.outputs)[0].set(dummy_file_path)
# ```

# https://docs.google.com/document/d/1YQWJOL8w5VGNcGW5TV4plDsh457P5HWktEuZhVd1v1o/edit#
# 
# https://www.tutorialspoint.com/python/python_multithreading.htm
# 
# https://towardsdatascience.com/introduction-to-priority-queues-in-python-83664d3178c3

# In[17]:


import numpy as np
import scipy
import scipy.sparse
import scipy.sparse.linalg
import threading
import time
import matplotlib.pylab as plt 
import json
from json import JSONEncoder
import random
import os

class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)
    
def jsonKeys2int(x):
    if isinstance(x, dict):
            return {int(k):v for k,v in x.items()}
    return 

def int_please_object_hook(obj):
    """If a value in obj is a string, try to convert it to an int"""
    rv = {}
    for k, v in obj.items():
        if isinstance(v, bytes) or isinstance(v, str):
            try:
                rv[k] = int(v)
            except ValueError:
                rv[k] = v
        else:
            rv[k] = v
    return rv


# In[18]:


class KindofPriorityQueue():
    
    def __init__(self):
        self.myqueue = []
        
    def pop(self):
        if not self.myqueue:
            return None
        return self.myqueue.pop()
    
    def insert(self,t,elem):
        retval=random.getrandbits(64)
        self.myqueue.append((t, retval, elem))
        self.myqueue.sort(reverse=True) 
        return retval
    
    def insert_with_index(self,t,elem,index):
        self.myqueue.append((t, index, elem))
        self.myqueue.sort(reverse=True)
        
    def delete(self, index):
        counter=0
        while counter<len(self.myqueue) and self.myqueue[counter][1]!=index:
            counter+=1
        if counter<len(self.myqueue):
            del self.myqueue[counter]
            
    def deleteall(self):
        self.myqueue = []
            
    def first(self):
        if not self.myqueue:
            return None
        return self.myqueue[-1]
    
    def get(self, index):
        retval=None
        counter=0
        while counter<len(self.myqueue) and self.myqueue[counter][1]!=index:
            counter+=1
        if counter<len(self.myqueue):
            retval=self.myqueue[counter]
        return retval
    
    def empty(self):
        return not self.myqueue


# In[19]:


class Controller:
    def __init__(self,tweakparam_key, initval, regulationparam_key, regulationparam_otherparams, setpoint, iteration_time, KP, KI, KD, controlled):
        self.iteration_time = iteration_time
        self.tweakparam_key = tweakparam_key
        self.initval = initval
        self.regulationparam_key = regulationparam_key
        self.regulationparam_otherparams = regulationparam_otherparams
        self.setpoint = setpoint
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.controlled = controlled
        self.errors=[]
        self.sets=[]
        self.controlledvals=[]
        
    def controllermain(self):
        error_prior = 0
        integral_prior = 0
        bias = 0 
        
        self.t = self.iteration_time
        
        recindex=self.controlled.record(self.regulationparam_key, self.iteration_time, self.regulationparam_otherparams)
        waittime=self.iteration_time
        waitindex=self.controlled.wait_for_me_at(waittime)
        newset=self.initval
        self.controlled.setnow(self.tweakparam_key, newset)
        self.controlled.start()
        self.errors=[]
        self.sets=[]
        self.controlledvals=[]
        lasttime=0
        while not self.controlled.finished():
            self.controlled.wait_for_time(waittime,1000)
            self.t=self.controlled.get_time()
            get1=self.controlled.get(str(recindex))
            if not get1:
                error1 = error_prior
                timestep=self.t-lasttime
                lasttime=self.t
                print('problem?')
            else:
                error1 = self.setpoint - get1[0][1]
                timestep=get1[0][0]-lasttime
                lasttime=get1[0][0]
            self.errors.append(error1)
            self.controlledvals.append(get1[0][1])
            integral = integral_prior + error1 * timestep
            derivative = (error1 - error_prior) / timestep
            output=self.KP*error1 + self.KI*integral + self.KD*derivative + bias
            newset=newset+output 
            self.controlled.setnow(self.tweakparam_key, newset)
            self.sets.append(newset)
            error_prior = error1
            integral_prior = integral
            waittime=waittime+self.iteration_time
            recindex=self.controlled.record(self.regulationparam_key,waittime,self.regulationparam_otherparams)
            waitindex=self.controlled.continue_until(waittime,waitindex)


# In[20]:


class Tsolver:
    def __init__(self, dx, n, Tinit, dt, Tsource, k, heatcapacity, sourcescale, tend, sidecar):
        self.T = Tinit;
        self.t = 0;
        self.dx = dx;
        self.n = n;
        self.dt = dt;
        self.Tsource = Tsource;
        self.k = k;
        self.sourcescale = sourcescale;
        self.tend = tend;
        self.sidecar = sidecar;
        self.heatcapacity=10

    def main(self):
        self.wait_for_start_signal()
        self.record(self.t)
        self.apply_set(self.t)
 
        while self.t<self.tend:
            self.record(self.t)
            self.wait_if_necessary(self.t)
            self.apply_set(self.t)  

            diffusion=self.k/(self.dx*self.dx) * (self.T[:n-2,1:n-1]+self.T[1:n-1,:n-2]+self.T[2:n,1:n-1]+self.T[1:n-1,2:n]-4*self.T[1:n-1,1:n-1]);
            self.T[1:n-1,1:n-1]=self.T[1:n-1,1:n-1]+self.dt*(self.sourcescale*self.Tsource+diffusion);
            self.t=self.t+self.dt;

        self.is_finished();
        return self.T;
    
    
    # SIDECAR
    def wait_if_necessary(self,t): #move what is possible into the sidecar
        self.sidecar.wait_if_necessary(t)

    # SIDECAR
    def wait_for_start_signal(self):
        self.sidecar.wait_for_start_signal()
        
    # SIDECAR
    def is_finished(self):
        self.record(float("inf"))
        self.sidecar.is_finished()

    # SIDECAR
    def record(self,t):
        recindex,recinfo=self.sidecar.is_there_something_to_record(t)
        while not (recindex is None):
            if recinfo[0]=='Tpoint':
                self.sidecar.record_for_me(recindex,t,self.T[recinfo[1][0],recinfo[1][1]])
            elif recinfo[0]=='Tvol':
                self.sidecar.record_for_me(recindex,t,self.T[recinfo[1][0]:recinfo[1][2],recinfo[1][1]:recinfo[1][3]])
            recindex,recinfo=self.sidecar.is_there_something_to_record(t)
    # INTERNAL
    def apply_set(self,t):
        setinfo=self.sidecar.is_there_something_to_set(t)
        while not (setinfo is None):
            if setinfo[0]=='Tsource':
                Tsource1=np.asarray(setinfo[1])
                if Tsource1.shape==self.Tsource.shape:
                    self.Tsource=Tsource1
            elif setinfo[0]=='SARsource':
                SARsource1=np.asarray(setinfo[1])
                if SARsource1.shape==self.Tsource.shape:
                    self.Tsource=SARsource1/self.heatcapacity
            elif setinfo[0]=='k':
                if setinfo[1]>0:
                    self.set_k(set1[1])
            elif setinfo[0]=='sourcescale':
                self.sourcescale=setinfo[1]
            elif setinfo[0]=='tend':
                self.tend=setinfo[1]
            setinfo=self.sidecar.is_there_something_to_set(t)
        


# In[21]:


class SideCar:
    def __init__(self,locks,syncpaths):
        self.locks=locks
        self.t=0;
        self.startsignal=False;
        self.endsignal=False;
        self.paused=True;
        #self.getsignal=False;
        self.waitqueue=KindofPriorityQueue()
        self.recordqueue=KindofPriorityQueue()
        self.setqueue=KindofPriorityQueue()
        self.records={}
        self.instructions=[]
        self.instructioncounter=0
        self.canbeset=[]
        self.canbegotten=[]
        self.syncpaths=syncpaths
        

    def can_be_set(self): # controllable parameters of the model
        return self.canbeset; 

    def is_there_something_to_set(self,t):
        self.t=t
        if (not self.setqueue.empty()) and self.setqueue.first()[0] <= t:
            return self.setqueue.pop()[2]
        else:
            return None
        
    def setnow(self,key,value):
        if(key in self.canbeset):
            self.setqueue.insert(self.t,(key,value))
            return 0
        return -1

    def set1(self,key,value,t):
        if(key in self.canbeset):
            self.setqueue.insert(t,(key,value))
            return 0
        return -1;

    def can_be_gotten(self): # observables of the model (similar to sensor)
        return self.canbegotten

    def record(self,key,timepoints,otherparams,index): # when and where to record observables
        if key in self.canbegotten:
            self.recordqueue.insert_with_index(timepoints,(key,otherparams),index) #xxx problem with more than one timepoint
            self.records[index]=[]
            
    def is_there_something_to_record(self,t):
        self.t=t
        if (not self.recordqueue.empty()) and self.recordqueue.first()[0] <= t:
            pop1=self.recordqueue.pop()
            return pop1[1],pop1[2]
        else:
            return None,None
        
    def record_for_me(self,recindex,t,item):
        self.records[recindex].append((t,item))
        self.t=t

    def wait_a_bit(self):
        time.sleep(0.05);
        
    def get(self,index):
        return self.records[index]

    #def get_current(self,key,otherparams):
    #    if key in self.can_be_gotten():
    #        index = self.recordqueue.insert(timepoints,(key,otherparams)) #xxx problem with more than one timepoint
    #        self.records[index]=[]
    #    counter=0
    #    maxcount=100
    #    while (not self.records[index]) and counter<maxcount: #xxx
    #        self.wait_a_bit()
    #        counter=counter+1;
    #    return self.records[index]

    def wait_for_me_at(self,t,index):
        self.waitqueue.insert_with_index(t,None,index)

    def continue_please(self,index):
        mywait=self.waitqueue.get(index)
        if mywait!=None:
            self.waitqueue.delete(index)
            if self.waitqueue.first()==None or self.waitqueue.first()[0]>self.t:
                self.release();
        

    def continue_until(self,t,index1,index2): # schedule your wait point for later
        self.wait_for_me_at(t,index1);
        self.continue_please(index2);
        return

    def start(self):
        self.startsignal=True;
        self.release();

    def pause(self):
        if (not self.paused) or (not self.startsignal):
            self.paused=True
            self.syncout()
        
    def release(self):
        if self.paused:
            self.syncin()
            self.paused=False 
            self.syncout()
            
    def syncin(self):
        inputdata = None
        if os.path.exists(self.syncpaths['insync']):
            with self.locks['insynclock']: # xxx timeout?
                with open(self.syncpaths['insync'], 'r') as f: 
                    inputdata = json.load(f)
        if inputdata != None:
            self.instructions=inputdata['instructions']
            self.executeInstructions()
            
    def syncout(self):
        outputdata={'t':self.t, 'endsignal':self.endsignal, 'paused':self.paused, 'records':self.records} # start?
        with self.locks['outsynclock']: # xxx timeout?
            with open(self.syncpaths['outsync'],"w") as output1:     
                json.dump(outputdata, output1, sort_keys=True, cls=NumpyArrayEncoder)
        
    def executeInstructions(self):
        l=len(self.instructions)
        
        while self.instructioncounter<l:
            i=self.instructioncounter
            inst=self.instructions[i]['inst']
            self.instructioncounter=i+1
            if inst=='setnow':
                self.setnow(self.instructions[i]['key'],self.instructions[i]['val'])
            elif inst=='set':
                self.set1(self.instructions[i]['key'],self.instructions[i]['val'],self.instructions[i]['t'])
            elif inst=='record':
                self.record(self.instructions[i]['key'],self.instructions[i]['timepoints'],self.instructions[i]['otherparams'],self.instructions[i]['index'])
            elif inst=='waitformeat':
                self.wait_for_me_at(self.instructions[i]['t'],self.instructions[i]['index'])
            elif inst=='continueplease':
                self.continue_please(self.instructions[i]['index'])
            elif inst=='continueuntil':
                self.continue_until(self.instructions[i]['t'],self.instructions[i]['index1'],self.instructions[i]['index2'])
            elif inst=='start':
                self.start()
                
    def wait_a_bit(self):
        time.sleep(0.05);
        
    def wait_if_necessary(self,t): #move what is possible into the sidecar
        self.syncout()
        while (not self.waitqueue.empty()) and self.waitqueue.first()[0] <= t:
            self.pause()
            self.wait_a_bit()
            self.syncin()
        self.release()

    def wait_for_start_signal(self):
        while not self.startsignal:
            self.wait_a_bit()
            self.syncin()
        self.release()
        
    def is_finished(self):
        self.waitqueue.deleteall();
        self.endsignal=True; #make function for this and the next line
        self.pause() # what happens if the sidecar is in the middle of executing the wait_for_pause; how about release synchronization


# In[22]:


class SideCarSatelite:
    def __init__(self,locks,syncpaths):
        self.locks=locks
        self.t=0;
        self.startsignal=False;
        self.endsignal=False;
        self.paused=True;
        self.records={}
        self.instructions=[]
        self.canbeset=[]
        self.canbegotten=[]
        self.syncpaths=syncpaths
 
    def can_be_set(self): # controllable parameters of the model
        return self.canbeset; 

    def setnow(self,key,value):
        self.instructions.append({'inst':'setnow','key':key,'val':value})

    def set1(self,key,value,t):
        self.instructions.append({'inst':'set','t':t,'key':key,'val':value})

    def can_be_gotten(self): # observables of the model (similar to sensor)
        return self.canbegotten

    def record(self,key,timepoints,otherparams): # when and where to record observables
        index = random.getrandbits(64)
        self.instructions.append({'inst':'record','timepoints':timepoints,'key':key,'otherparams':otherparams,'index':index})
        return index
                
    def wait_a_bit(self):
        time.sleep(0.05);
        
    def get(self,index):
        return self.records[index]
        
    def wait_for_time(self,waittime,maxcount):
        counter=0;
        while self.t<waittime and counter<maxcount:
            self.syncout()
            self.syncin()
            self.wait_a_bit()
            counter+=1
        if self.t<waittime:
            print('timeout')

    def get_time(self):
        return t;

    def wait_for_me_at(self,t):
        index = random.getrandbits(64)
        self.instructions.append({'inst':'waitformeat','t':t,'index':index})
        return index 

    def continue_please(self,index):
        self.instructions.append({'inst':'continueplease','index':index})
        self.release()
        
    def continue_until(self,t,index): # schedule your wait point for later
        index1 = random.getrandbits(64)
        self.instructions.append({'inst':'continueuntil','t':t,'index1':index1,'index2':index})
        self.release()
        return index1;

    def start(self):
        self.instructions.append({'inst':'start'})
        self.release()

    def finished(self):
        return self.endsignal;
        
    def release(self):
        if self.paused:
            self.syncin()
            self.paused=False #syncout of the pause?
            self.syncout()
            
    def syncout(self):
        if os.path.exists(self.syncpaths['outsync']):
            with self.locks['outsynclock']: # xxx timeout?
                with open(self.syncpaths['outsync'], 'r') as f: 
                    inputdata = json.load(f)
            self.t=inputdata['t']
            self.endsignal=inputdata['endsignal']
            self.paused=inputdata['paused']
            self.records=inputdata['records']
            
    def syncin(self):
        outputdata={'instructions':self.instructions}
        with self.locks['insynclock']: # xxx timeout?
            with open(self.syncpaths['insync'],"w") as output1:     
                json.dump(outputdata, output1, sort_keys=True, cls=NumpyArrayEncoder)


# In[23]:


class TSolverSideCar(SideCar):
    def __init__(self,locks,syncpaths):
        SideCar.__init__(self,locks,syncpaths)
        self.canbeset=self.can_be_set()
        self.canbegotten=self.can_be_gotten()
        
    def can_be_set(self): # controllable parameters of the model
        return ['Tsource', 'SARsource', 'k', 'sourcescale', 'tend']; 
    
    def can_be_gotten(self): # observables of the model (similar to sensor)
        return ['Tpoint', 'Tvol'];


# In[24]:


class TSolverSideCarSatelite(SideCarSatelite):
    def __init__(self,locks,syncpaths):
        SideCarSatelite.__init__(self,locks,syncpaths)
        self.canbeset=self.can_be_set()
        self.canbegotten=self.can_be_gotten()
        
    def can_be_set(self): # controllable parameters of the model
        return TSolverSideCar.can_be_set(self); 
    
    def can_be_gotten(self): # observables of the model (similar to sensor)
        return TSolverSideCar.can_be_gotten(self);


# In[25]:


class TsolverThread(threading.Thread): 
    def __init__(self, dx, n, Tinit, dt, Tsource, k, heatcapacity, sourcescale, tend, sidecar):
        threading.Thread.__init__(self)
        self.myTsolver=Tsolver(dx, n, Tinit, dt, Tsource, k, heatcapacity, sourcescale, tend, sidecar)
        self.name='Tsolver'
    def run(self):
        print("Starting ",self.name)
        T=self.myTsolver.main()
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.set_aspect('equal')
        plt.imshow(T)
        plt.colorbar()
        plt.show() #causes floating point error. why?
        print("Exiting ",self.name)


# In[26]:


class TsolverSidecarThread(threading.Thread): 
    def __init__(self,locks,syncpaths):
        threading.Thread.__init__(self)
        self.myTSolverSideCar=TSolverSideCar(locks,syncpaths)
        self.name='TsolverSidecar'
        self.stop=False
    def run(self):
        print("Starting ",self.name)
        while not self.stop:
            time.sleep(0.1)
        print("Exiting ",self.name)


# In[27]:


class TsolverSidecarSateliteThread(threading.Thread): 
    def __init__(self,locks,syncpaths):
        threading.Thread.__init__(self)
        self.myTSolverSideCarSatelite=TSolverSideCarSatelite(locks,syncpaths)
        self.name='TsolverSidecarSatelite'
        self.stop=False
    def run(self):
        print("Starting ",self.name)
        while not self.stop:
            time.sleep(0.1)
        print("Exiting ",self.name)


# In[28]:


class ControllerThread(threading.Thread): 
    def __init__(self,tweakparam_key, initval, regulationparam_key, regulationparam_otherparams, setpoint, iteration_time, KP, KI, KD, controlled):
        threading.Thread.__init__(self)
        self.myController=Controller(tweakparam_key, initval, regulationparam_key, regulationparam_otherparams, setpoint, iteration_time, KP, KI, KD, controlled)
        self.name='Controller'
    def run(self):
        print("Starting ",self.name)
        self.myController.controllermain()
        print("Exiting ",self.name)


# In[29]:


syncpathsT = {
    "outsync":"outputs/output_2/insyncT.json",
    "insync":"outputs/output_1/outsyncT.json"    
}

if os.path.exists(syncpathsT['insync']):
      os.remove(syncpathsT['insync'])
if os.path.exists(syncpathsT['outsync']):
      os.remove(syncpathsT['outsync'])


# In[30]:


n=20; Tinit=np.zeros((n,n), float); Tsource=np.ones((n-2,n-2), float); 
locks = {
    "outsynclock":threading.Lock(),
    "insynclock":threading.Lock()
}

sidecar1=TSolverSideCar(locks,syncpathsT)
sidecar1.startsignal=True
myTsolver1=Tsolver(1, n, Tinit, 0.1, Tsource, 1, 10, 1, 500, sidecar1)
T=myTsolver1.main()   
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.set_aspect('equal')
plt.imshow(T)
plt.colorbar()
plt.show()
TSolverSideCar.stop=True


# In[31]:


if os.path.exists(syncpathsT['insync']):
      os.remove(syncpathsT['insync'])
if os.path.exists(syncpathsT['outsync']):
      os.remove(syncpathsT['outsync'])


# In[32]:


n=20; Tinit=np.zeros((n,n), float); Tsource=np.ones((n-2,n-2), float); 

locks = {
    "outsynclock":threading.Lock(),
    "insynclock":threading.Lock()
}

thread1a=TsolverSidecarThread(locks,syncpathsT)
thread1b=TsolverSidecarSateliteThread(locks,syncpathsT)
thread2=TsolverThread(1, n, Tinit, 0.1, Tsource, 1, 10, 1, 500, thread1a.myTSolverSideCar)
thread3=ControllerThread('sourcescale', 1, 'Tpoint', [10,10], 4, 10, 0.01, 0, 0, thread1b.myTSolverSideCarSatelite)#tweakparam_key, regulationparam_key, regulationparam_otherparams, setpoint, iteration_time, KP, KI, KD

# Start new Threads
thread1a.start()
thread1b.start()
thread2.start()
thread3.start()

threads = []
threads.append(thread2)
threads.append(thread3)

# Wait for all threads to complete
for t in threads:
    t.join()
thread1a.stop=True
thread1b.stop=True

fig, (ax1, ax2,ax3) = plt.subplots(3)
ax1.plot(thread3.myController.sets)
ax1.set_title('set value')
ax2.plot(thread3.myController.errors)
ax2.set_title('errors')
ax3.plot(thread3.myController.controlledvals)
ax3.set_title('controlled value')
print("Exiting Main Thread")


# to do:
# - more than one recording point

# f(123)

# example 2: coupled simulator

# In[33]:


class EMsolver:
    def __init__(self, n, EMinit, dt, EMsource, Tinit, tend, sigma0, sigmaT, sidecar):
        self.EM = EMinit;
        self.t = 0
        self.n = n
        self.dt = dt
        self.EMsource = EMsource.flatten()
        self.sigma0 = sigma0;
        self.sigmaT = sigmaT;
        self.sigma = self.sigma_calc(Tinit)
        self.tend = tend;
        self.sidecar = sidecar;
        self.V = np.zeros([n,n])
        self.SAR = np.zeros([n,n])

    def solveV(self):
        nrcoefs=self.n*self.n+4*self.n*(self.n-1)
        n2=self.n*self.n
        row=np.zeros(nrcoefs, dtype=int)
        col=np.zeros(nrcoefs, dtype=int)
        data=np.zeros(nrcoefs)
        
        counter=0
        mysigma=self.sigma[0,0]
        row[1]=0
        col[1]=1
        data[1]=mysigma*self.sigma[0,1]/(mysigma+self.sigma[0,1])
        row[2]=0
        col[2]=self.n
        data[2]=mysigma*self.sigma[1,0]/(mysigma+self.sigma[1,0])
        row[0]=0
        col[0]=0
        data[0]=-(4*mysigma+data[1]+data[2])
        counter+=3
        for j in range(1,self.n-1):
            mysigma=self.sigma[0,j]
            row[counter+1]=j
            col[counter+1]=j+1
            data[counter+1]=mysigma*self.sigma[0,j+1]/(mysigma+self.sigma[0,j+1])
            row[counter+2]=j
            col[counter+2]=j+self.n
            data[counter+2]=mysigma*self.sigma[1,j]/(mysigma+self.sigma[1,j])
            row[counter+3]=j
            col[counter+3]=j-1
            data[counter+3]=mysigma*self.sigma[0,j-1]/(mysigma+self.sigma[0,j-1])
            row[counter]=j
            col[counter]=j
            data[counter]=-(2*mysigma+data[counter+1]+data[counter+2]+data[counter+3])
            counter+=4
        j=self.n-1
        mysigma=self.sigma[0,j]
        row[counter+1]=j
        col[counter+1]=j+self.n
        data[counter+1]=mysigma*self.sigma[1,j]/(mysigma+self.sigma[1,j])
        row[counter+2]=j
        col[counter+2]=j-1
        data[counter+2]=mysigma*self.sigma[0,j-1]/(mysigma+self.sigma[0,j-1])
        row[counter]=j
        col[counter]=j
        data[counter]=-(4*mysigma+data[counter+1]+data[counter+2])
        counter+=3
        myindex=self.n
        for i in range(1,self.n-1):
            mysigma=self.sigma[i,0]
            row[counter+1]=myindex
            col[counter+1]=myindex+1
            data[counter+1]=mysigma*self.sigma[i,1]/(mysigma+self.sigma[i,1])
            row[counter+2]=myindex
            col[counter+2]=myindex+self.n
            data[counter+2]=mysigma*self.sigma[i+1,0]/(mysigma+self.sigma[i+1,0])
            row[counter+3]=myindex
            col[counter+3]=myindex-self.n
            data[counter+3]=mysigma*self.sigma[i-1,0]/(mysigma+self.sigma[i-1,0])
            row[counter]=myindex
            col[counter]=myindex
            data[counter]=-(2*mysigma+data[counter+1]+data[counter+2]+data[counter+3])
            counter+=4
            myindex+=1
            for j in range(1,self.n-1):
                mysigma=self.sigma[i,j]
                row[counter+1]=myindex
                col[counter+1]=myindex+1
                data[counter+1]=mysigma*self.sigma[i,j+1]/(mysigma+self.sigma[i,j+1])
                row[counter+2]=myindex
                col[counter+2]=myindex+self.n
                data[counter+2]=mysigma*self.sigma[i+1,j]/(mysigma+self.sigma[i+1,j])
                row[counter+3]=myindex
                col[counter+3]=myindex-self.n
                data[counter+3]=mysigma*self.sigma[i-1,j]/(mysigma+self.sigma[i-1,j])
                row[counter+4]=myindex
                col[counter+4]=myindex-1
                data[counter+4]=mysigma*self.sigma[i,j-1]/(mysigma+self.sigma[i,j-1])
                row[counter]=myindex
                col[counter]=myindex
                data[counter]=-(data[counter+1]+data[counter+2]+data[counter+3]+data[counter+4])
                counter+=5
                myindex+=1
            j=self.n-1
            mysigma=self.sigma[i,j]
            row[counter+1]=myindex
            col[counter+1]=myindex+self.n
            data[counter+1]=mysigma*self.sigma[i+1,j]/(mysigma+self.sigma[i+1,j])
            row[counter+2]=myindex
            col[counter+2]=myindex-self.n
            data[counter+2]=mysigma*self.sigma[i-1,j]/(mysigma+self.sigma[i-1,j])
            row[counter+3]=myindex
            col[counter+3]=myindex-1
            data[counter+3]=mysigma*self.sigma[i,j-1]/(mysigma+self.sigma[i,j-1])
            row[counter]=myindex
            col[counter]=myindex
            data[counter]=-(2*mysigma+data[counter+1]+data[counter+2]+data[counter+3])
            counter+=4
            myindex+=1
        i=self.n-1
        mysigma=self.sigma[i,0]
        row[counter+1]=myindex
        col[counter+1]=myindex+1
        data[counter+1]=mysigma*self.sigma[i,1]/(mysigma+self.sigma[i,1])
        row[counter+2]=myindex
        col[counter+2]=myindex-self.n
        data[counter+2]=mysigma*self.sigma[i-1,0]/(mysigma+self.sigma[i-1,0])
        row[counter]=myindex
        col[counter]=myindex
        data[counter]=-(4*mysigma+data[counter+1]+data[counter+2])
        counter+=3
        myindex+=1
        for j in range(1,self.n-1):
            mysigma=self.sigma[i,j]
            row[counter+1]=myindex
            col[counter+1]=myindex+1
            data[counter+1]=mysigma*self.sigma[i,j+1]/(mysigma+self.sigma[i,j+1])
            row[counter+2]=myindex
            col[counter+2]=myindex-self.n
            data[counter+2]=mysigma*self.sigma[i-1,j]/(mysigma+self.sigma[i-1,j])
            row[counter+3]=myindex
            col[counter+3]=myindex-1
            data[counter+3]=mysigma*self.sigma[i,j-1]/(mysigma+self.sigma[i,j-1])
            row[counter]=myindex
            col[counter]=myindex
            data[counter]=-(2*mysigma+data[counter+1]+data[counter+2]+data[counter+3])
            counter+=4
            myindex+=1
        j=self.n-1
        mysigma=self.sigma[i,j]
        row[counter+1]=myindex
        col[counter+1]=myindex-self.n
        data[counter+1]=mysigma*self.sigma[i-1,j]/(mysigma+self.sigma[i-1,j])
        row[counter+2]=myindex
        col[counter+2]=myindex-1
        data[counter+2]=mysigma*self.sigma[i,j-1]/(mysigma+self.sigma[i,j-1])
        row[counter]=myindex
        col[counter]=myindex
        data[counter]=-(4*mysigma+data[counter+1]+data[counter+2])
        
        coefmatrix=scipy.sparse.csr_matrix( (data,(row,col)), shape=(n2,n2) )
        self.V = scipy.sparse.linalg.spsolve(coefmatrix, -self.EMsource).reshape((self.n, self.n))        
        
    def computeSAR(self):
        self.SAR=self.sigma*self.V
        #xxx self.sigma*np.abs(self.E[rec1[1][0]:rec1[1][2],rec1[1][1]:rec1[1][3]])^2/(2*self.rho))
                                                      
    def sigma_calc(self,T):
        return self.sigma0+T*self.sigmaT
    
    def main(self):
        self.wait_for_start_signal()
        self.record(self.t)
        self.apply_set(self.t)
        
        while self.t<self.tend:
            self.record(self.t)
            self.wait_if_necessary(self.t)
            self.apply_set(self.t)  

            self.solveV()
            self.computeSAR()
            self.t=self.t+self.dt;
        
        self.is_finished()
        
        return self.SAR
 
    def wait_if_necessary(self,t): #move what is possible into the sidecar
        self.sidecar.wait_if_necessary(t)

    def wait_for_start_signal(self):
        self.sidecar.wait_for_start_signal()

    def is_finished(self):
        self.record(float("inf"))
        self.sidecar.is_finished()

    def record(self,t):
        recindex,recinfo=self.sidecar.is_there_something_to_record(t)
        while not (recindex is None):
            if recinfo[0]=='SARvol':
                self.sidecar.record_for_me(recindex,t,self.SAR[recinfo[1][0]:recinfo[1][2],recinfo[1][1]:recinfo[1][3]])
            recindex,recinfo=self.sidecar.is_there_something_to_record(t)

    def apply_set(self,t):
        setinfo=self.sidecar.is_there_something_to_set(t)
        while not (setinfo is None):
            if setinfo[0]=='sigma':
                sigma1=np.asarray(setinfo[1])
                if sigma1.shape==self.sigma.shape:
                    self.sigma=sigma1
            elif setinfo[0]=='T':
                T1=np.asarray(setinfo[1])
                if T1.shape==self.sigma.shape:
                    self.sigma=self.sigma_calc(T1)
            elif setinfo[0]=='tend':
                self.tend=setinfo[1]
            setinfo=self.sidecar.is_there_something_to_set(t)                


# In[34]:


class EMSolverSideCar(SideCar):
    def __init__(self,locks,syncpaths):
        SideCar.__init__(self,locks,syncpaths)
        self.canbeset=self.can_be_set()
        self.canbegotten=self.can_be_gotten()
        
    def can_be_set(self): # controllable parameters of the model
        return ['sigma', 'T', 'tend'];
    
    def can_be_gotten(self): # observables of the model (similar to sensor)
        return ['SARvol'];


# In[35]:


class EMSolverSideCarSatelite(SideCarSatelite):
    def __init__(self,locks,syncpaths):
        SideCarSatelite.__init__(self,locks,syncpaths)
        self.canbeset=self.can_be_set()
        self.canbegotten=self.can_be_gotten()
        
    def can_be_set(self): # controllable parameters of the model
        return EMSolverSideCar.can_be_set(self);
    
    def can_be_gotten(self): # observables of the model (similar to sensor)
        return EMSolverSideCar.can_be_gotten(self);


# In[36]:


class EMsolverThread(threading.Thread): 
    def __init__(self, n, EMinit, dt, EMsource, Tinit, tend, sigma0, sigmaT, sidecar):
        threading.Thread.__init__(self)
        self.myEMsolver=EMsolver(n, EMinit, dt, EMsource, Tinit, tend, sigma0, sigmaT, sidecar)
        self.name='EMsolver'
    def run(self):
        print("Starting ",self.name)
        SAR=self.myEMsolver.main()
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.set_aspect('equal')
        plt.imshow(SAR)
        plt.colorbar()
        plt.show()
        print("Exiting ",self.name)


# In[37]:


class EMsolverSidecarThread(threading.Thread): 
    def __init__(self,locks,syncpaths):
        threading.Thread.__init__(self)
        self.myEMSolverSideCar=EMSolverSideCar(locks,syncpaths)
        self.name='EMsolverSidecar'
        self.stop=False
    def run(self):
        print("Starting ",self.name)
        while not self.stop:
            time.sleep(0.1)
        print("Exiting ",self.name)


# In[38]:


class EMsolverSidecarSateliteThread(threading.Thread): 
    def __init__(self,locks,syncpaths):
        threading.Thread.__init__(self)
        self.myEMSolverSideCarSatelite=EMSolverSideCarSatelite(locks,syncpaths)
        self.name='EMsolverSidecarSatelite'
        self.stop=False
    def run(self):
        print("Starting ",self.name)
        while not self.stop:
            time.sleep(0.1)
        print("Exiting ",self.name)


# In[39]:


syncpathsEM = {
    "outsync":"outputs/output_2/insyncEM.json",
    "insync":"outputs/output_1/outsyncEM.json"    
}

if os.path.exists(syncpathsEM['insync']):
      os.remove(syncpathsEM['insync'])
if os.path.exists(syncpathsEM['outsync']):
      os.remove(syncpathsEM['outsync'])


# In[40]:


n=20; 
Tinit=np.zeros((n,n), float)
EMsource=np.zeros((n,n), float)
EMsource[int(n/3),int(n/4)]=1
EMinit=np.zeros((n,n), float)
locks = {
    "outsynclock":threading.Lock(),
    "insynclock":threading.Lock()
}
sidecar1=EMSolverSideCar(locks,syncpathsEM)
sidecar1.startsignal=True
myEMsolver1=EMsolver(n, EMinit, 1, EMsource, Tinit, 1.1, 1, 0, sidecar1)
SAR=myEMsolver1.main()
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.set_aspect('equal')
plt.imshow(SAR)
plt.colorbar()
plt.show()
EMSolverSideCar.stop=True


# In[41]:


class EM_T_coupler:
    def __init__(self,sidecarsateliteEM,sidecarsateliteT,EMsetparam_key,EMgetparam_key,EMgetparam_otherparams,Tsetparam_key,Tgetparam_key,Tgetparam_otherparams,coupling_interval):
        self.sidecarsateliteEM=sidecarsateliteEM
        self.sidecarsateliteT=sidecarsateliteT
        self.EMsetparam_key=EMsetparam_key
        self.Tsetparam_key=Tsetparam_key
        self.EMgetparam_key=EMgetparam_key
        self.EMgetparam_otherparams=EMgetparam_otherparams
        self.Tgetparam_key=Tgetparam_key
        self.Tgetparam_otherparams=Tgetparam_otherparams
        self.coupling_interval=coupling_interval
        self.Tstored=[]
        self.EMstored=[]
        
    def main(self):
        time=0
        recindexEM=self.sidecarsateliteEM.record(self.EMgetparam_key, self.coupling_interval, self.EMgetparam_otherparams)
        recindexT=self.sidecarsateliteT.record(self.Tgetparam_key, self.coupling_interval, self.Tgetparam_otherparams)

        waitindexEM=self.sidecarsateliteEM.wait_for_me_at(self.coupling_interval)
        waitindexT=self.sidecarsateliteT.wait_for_me_at(self.coupling_interval)
        self.sidecarsateliteEM.start()
        self.sidecarsateliteT.start()
        nexttime=self.coupling_interval
        while not (self.sidecarsateliteEM.finished() or self.sidecarsateliteT.finished()): #xxx correct?
            self.sidecarsateliteEM.wait_for_time(nexttime,1000)
            self.sidecarsateliteT.wait_for_time(nexttime,1000)
            SAR=np.asarray(self.sidecarsateliteEM.get(str(recindexEM))[0][1])
            T=np.asarray(self.sidecarsateliteT.get(str(recindexT))[0][1])
            self.sidecarsateliteEM.setnow(self.EMsetparam_key, T)
            self.sidecarsateliteT.setnow(self.Tsetparam_key, SAR)
            self.Tstored.append(T)
            self.EMstored.append(SAR)
            nexttime=nexttime+self.coupling_interval
            recindexEM=self.sidecarsateliteEM.record(self.EMgetparam_key, nexttime, self.EMgetparam_otherparams)
            recindexT=self.sidecarsateliteT.record(self.Tgetparam_key, nexttime, self.Tgetparam_otherparams)    
            waitindexEM=self.sidecarsateliteEM.continue_until(nexttime,waitindexEM)    
            waitindexT=self.sidecarsateliteT.continue_until(nexttime,waitindexT)


# In[42]:


class EM_T_couplerThread(threading.Thread): 
    def __init__(self,sidecarsateliteEM,sidecarsateliteT,EMsetparam_key,EMgetparam_key,EMgetparam_otherparams,Tsetparam_key,Tgetparam_key,Tgetparam_otherparams,coupling_interval):
        threading.Thread.__init__(self)
        self.myCoupler=EM_T_coupler(sidecarsateliteEM,sidecarsateliteT,EMsetparam_key,EMgetparam_key,EMgetparam_otherparams,Tsetparam_key,Tgetparam_key,Tgetparam_otherparams,coupling_interval)
        self.name='EM_T_coupler'
    def run(self):
        print("Starting ",self.name)
        self.myCoupler.main()
        print("Exiting ",self.name)


# In[43]:


if os.path.exists(syncpathsT['insync']):
      os.remove(syncpathsT['insync'])
if os.path.exists(syncpathsT['outsync']):
      os.remove(syncpathsT['outsync'])
if os.path.exists(syncpathsEM['insync']):
      os.remove(syncpathsEM['insync'])
if os.path.exists(syncpathsEM['outsync']):
      os.remove(syncpathsEM['outsync'])


# In[44]:


n=20; 
Tinit=np.zeros((n,n), float)
EMsource=np.zeros((n,n), float)
EMsource[int(n/3),int(n/4)]=1
EMinit=np.zeros((n,n), float)
locks = {
    "outsynclock":threading.Lock(),
    "insynclock":threading.Lock()
}
# sidecar1=EMSolverSideCar(locks,syncpathsEM)
# sidecar1.startsignal=True
# myEMsolver1=EMsolver(n, EMinit, 1, EMsource, Tinit, 1.1, 1, 0, sidecar1)
# SAR=myEMsolver1.main()
# fig = plt.figure()
# ax = fig.add_subplot(1,1,1)
# ax.set_aspect('equal')
# plt.imshow(SAR)
# plt.colorbar()
# plt.show()
# EMSolverSideCar.stop=True


# In[45]:


n=20; Tinit=np.zeros((n,n), float); Tsource=np.ones((n-2,n-2), float); 

locksEM = {
    "outsynclock":threading.Lock(),
    "insynclock":threading.Lock()
}
locksT = {
    "outsynclock":threading.Lock(),
    "insynclock":threading.Lock()
}

threadT1a=TsolverSidecarThread(locksT,syncpathsT)
threadT1b=TsolverSidecarSateliteThread(locksT,syncpathsT)
threadEM1a=EMsolverSidecarThread(locksEM,syncpathsEM)
threadEM1b=EMsolverSidecarSateliteThread(locksEM,syncpathsEM)
threadT2=TsolverThread(1, n, Tinit, 0.1, Tsource, 1, 10, 1, 5, threadT1a.myTSolverSideCar)
threadEM2=EMsolverThread(n, EMinit, 1, EMsource, Tinit, 5, 1, 5, threadEM1a.myEMSolverSideCar)
thread3Coupling=EM_T_couplerThread(threadEM1b.myEMSolverSideCarSatelite,threadT1b.myTSolverSideCarSatelite,'T','SARvol',[1,1,n-1,n-1],'SARsource','Tvol',[0,0,n,n],1)


# Start new Threads
threadT1a.start()
threadT1b.start()
threadEM1a.start()
threadEM1b.start()
threadT2.start()
threadEM2.start()
thread3Coupling.start()

threads = []
threads.append(threadT2)
threads.append(threadEM2)
threads.append(thread3Coupling)

# Wait for all threads to complete
for t in threads:
    t.join()
threadT1a.stop=True
threadT1b.stop=True
threadEM1a.stop=True
threadEM1b.stop=True

print("Exiting Main Thread")


# In[46]:


plt.figure(figsize=(15,5))
plt.subplot(1, len(thread3Coupling.myCoupler.Tstored), 1)
print('Temperature evolution')
for i in range(len(thread3Coupling.myCoupler.Tstored)):
    plt.subplot(1, 5, i+1)
    plt.imshow(thread3Coupling.myCoupler.Tstored[i], vmin=0, vmax=1.3)
plt.show()
plt.figure(figsize=(15,5))
plt.subplot(1, len(thread3Coupling.myCoupler.Tstored), 1)
print('EM evolution')
for i in range(len(thread3Coupling.myCoupler.EMstored)):
    plt.subplot(1, 5, i+1)
    plt.imshow(thread3Coupling.myCoupler.EMstored[i], vmin=0, vmax=1.5)
plt.show()


# In[ ]:




