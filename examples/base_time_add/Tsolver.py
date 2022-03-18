

import requests
from lockfile import LockFile
from osparc_control import ControlInterface
import numpy as np
import scipy
import threading
import time
import matplotlib.pyplot as plt 
import json
import random
import os

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


# In[5]:


class Tsolver:
    def __init__(self, dx, n, Tinit, dt, Tsource, k, sourcescale, tend, sidecar):
        self.T = Tinit;
        self.t = 0;
        self.dx = dx;
        self.n = n;
        self.Tinit = Tinit;
        self.dt = dt;
        self.Tsource = Tsource;
        self.k = k;
        self.sourcescale = sourcescale;
        self.tend = tend;
        self.sidecar = sidecar;

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
    
    def wait_a_bit(self):
        time.sleep(0.05);
        
    def wait_if_necessary(self,t): #move what is possible into the sidecar
        self.sidecar.syncout()
        while (not self.sidecar.waitqueue.empty()) and self.sidecar.waitqueue.first()[0] <= t:
            self.sidecar.pause()
            self.wait_a_bit()
            self.sidecar.syncin()
        self.sidecar.release()

    def wait_for_start_signal(self):
        while not self.sidecar.startsignal:
            self.wait_a_bit()
            self.sidecar.syncin()
        self.sidecar.release()

    def is_finished(self):
        self.record(float("inf"))
        self.sidecar.waitqueue.deleteall();
        self.sidecar.endsignal=True; #make function for this and the next line
        self.sidecar.pause() # what happens if the sidecar is in the middle of executing the wait_for_pause; how about release synchronization

    def record(self,t):
        while (not self.sidecar.recordqueue.empty()) and self.sidecar.recordqueue.first()[0] <= t:
            pop1=self.sidecar.recordqueue.pop()
            recindex=pop1[1]
            rec1=pop1[2]
            if rec1[0]=='Tpoint':
                self.sidecar.records[recindex].append((t,self.T[rec1[1][0],rec1[1][1]]))
            elif rec1[0]=='Tvol':
                self.sidecar.records[recindex].append((t,self.T[rec1[1][0]:rec1[1][2],rec1[1][1]:rec1[1][3]]))
        self.sidecar.t=t

    def apply_set(self,t):
        while (not self.sidecar.setqueue.empty()) and self.sidecar.setqueue.first()[0] <= t:
            set1=self.sidecar.setqueue.pop()[2]
            if set1[0]=='Tsource':
                if set1[1].shape==Tsource.shape:
                    self.Tsource=set1[1]
            elif set1[0]=='SARsource':
                if set1[1].shape==Tsource.shape:
                    self.Tsource=set1[1]/heatcapacity
            elif set1[0]=='k':
                if set1[1]>0:
                    self.set_k(set1[1])
            elif set1[0]=='sourcescale':
                self.sourcescale=set1[1]
            elif set1[0]=='tend':
                self.tend=set1[1]


# In[6]:


class SideCar:
    def __init__(self, interface):
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
        self.interface=interface

    def can_be_set(self): # controllable parameters of the model
        return self.canbeset; 

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

    def wait_a_bit(self):
        time.sleep(0.05);
        
    def get(self,index):
        return self.records[index]

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
        ##### CHANGE #########
        resp = requests.get(self.sidecar_url)
        if resp.status_code==200:
            try: 
                inputdata = json.loads(resp.json()['content'])

            except:
                print("problem reading data from " + self.sidecar_url)
        if inputdata != None:
            self.instructions=inputdata['instructions']
            self.executeInstructions()
        ##### CHANGE #########    
            
    def syncout(self):
        ##### CHANGE #########
        outputdata={'t':self.t, 'endsignal':self.endsignal, 'paused':self.paused, 'records':self.records} # start?
        with LockFile(outsyncpath):
#         logger_out.info(json.dumps(outputdata, sort_keys=True))
#         with self.locks['outsynclock']: # xxx timeout?
            with open(outsyncpath,"w") as output1:     
                json.dump(outputdata, output1, sort_keys=True)
        ##### CHANGE #########
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


# In[7]:


class SideCarSatelite:
    def __init__(self):
        self.t=0;
        self.startsignal=False;
        self.endsignal=False;
        self.paused=True;
        self.records={}
        self.instructions=[]
        self.canbeset=[]
        self.canbegotten=[]
 
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
        resp = requests.get(sidecarsatelite_url)
        if resp.status_code==200:
            inputdata = json.loads(resp.json()['content'])
#         if os.path.exists(outsyncpath):
#             with LockFile(outsyncpath):
# #             with self.locks['outsynclock']: # xxx timeout?
#                 with open(outsyncpath, 'r') as f: 
#                     inputdata = json.load(f)
            self.t=inputdata['t']
            self.endsignal=inputdata['endsignal']
            self.paused=inputdata['paused']
            self.records=inputdata['records']
            
    def syncin(self):
        outputdata={'instructions':self.instructions}
        with LockFile(insyncpath):
#         logger_out.info(json.dumps(outputdata, sort_keys=True))
#         import pdb; pdb.set_trace()
#         with self.locks['insynclock']: # xxx timeout?
            with open(insyncpath,"w") as output1:     
                json.dump(outputdata, output1, sort_keys=True)


# In[8]:


class TSolverSideCar(SideCar):
    def __init__(self, interface ):
        SideCar.__init__(self, interface)
        self.canbeset=self.can_be_set()
        self.canbegotten=self.can_be_gotten()
        
    def can_be_set(self): # controllable parameters of the model
        return ['Tsource', 'SARsource', 'k', 'sourcescale', 'tend']; 
    
    def can_be_gotten(self): # observables of the model (similar to sensor)
        return ['Tpoint', 'Tvol'];


# In[9]:


class TSolverSideCarSatelite(SideCarSatelite):
    def __init__(self ):
        SideCarSatelite.__init__(self )
        self.canbeset=self.can_be_set()
        self.canbegotten=self.can_be_gotten()
        
    def can_be_set(self): # controllable parameters of the model
        return ['Tsource', 'SARsource', 'k', 'sourcescale', 'tend']; 
    
    def can_be_gotten(self): # observables of the model (similar to sensor)
        return ['Tpoint', 'Tvol'];


# In[10]:


class TsolverThread(threading.Thread): 
    def __init__(self, dx, n, Tinit, dt, Tsource, k, sourcescale, tend, sidecar):
        threading.Thread.__init__(self)
        self.myTsolver=Tsolver(dx, n, Tinit, dt, Tsource, k, sourcescale, tend, sidecar)
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


# In[11]:


class TsolverSidecarThread(threading.Thread): 
    def __init__(self, interface ):
        threading.Thread.__init__(self)
        self.myTSolverSideCar=TSolverSideCar(interface)
        self.name='TsolverSidecar'
        self.stop=False
    def run(self):
        print("Starting ",self.name)
        while not self.stop:
            time.sleep(0.1)
        print("Exiting ",self.name)


# In[12]:


class TsolverSidecarSateliteThread(threading.Thread): 
    def __init__(self ):
        threading.Thread.__init__(self)
        self.myTSolverSideCarSatelite=TSolverSideCarSatelite()
        self.name='TsolverSidecarSatelite'
        self.stop=False
    def run(self):
        print("Starting ",self.name)
        while not self.stop:
            time.sleep(0.1)
        print("Exiting ",self.name)




