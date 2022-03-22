

from lockfile import LockFile
import requests

import numpy as np
import scipy
import threading
import time
import matplotlib.pylab as plt 
import json
import random
import os


# insyncpath = "test_controller/insync.json"
# outsyncpath = "test_controller/outsync.json"
# sidecarsatelite_url = "https://osparc-master.speag.com/x/9bd7ee34-f2d8-4d6e-ac6d-754a93e8623c" + "/api/contents/outsync.json"
# sidecar_url = "https://osparc-master.speag.com/x/9bd7ee34-f2d8-4d6e-ac6d-754a93e8623c" + "/api/contents/insync.json"



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


# In[4]:


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
            get1=self.controlled.get(recindex)
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
            print(self.controlled.finished())


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



######## CHANGE THIS ########################    
#         
    def syncin(self):
        inputdata = None
        resp = requests.get(sidecar_url)
#         if resp.status_code==200:
#             inputdata = json.loads(resp.json()['content'])
        if os.path.exists(insyncpath):
#             with self.locks['insynclock']: # xxx timeout?
            with LockFile(insyncpath):
                with open(insyncpath, 'r') as f: 
                    inputdata = json.load(f)
        if inputdata != None:
            self.instructions=inputdata['instructions']
            self.executeInstructions()
            
    def syncout(self):
        outputdata={'t':self.t, 'endsignal':self.endsignal, 'paused':self.paused, 'records':self.records} # start?
        request_id = self.interface.request_without_reply(
            "command_generic", params=outputdata
        )   
        print("sent " + str(request_id))             
#         with LockFile(outsyncpath):
#             with open(outsyncpath,"w") as output1:     
#                 json.dump(outputdata, output1, sort_keys=True)

######## CHANGE THIS ########################  
 
     
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
    def __init__(self, interface):
        self.t=0;
        self.startsignal=False;
        self.endsignal=False;
        self.paused=True;
        self.records={}
        self.instructions=[]
        self.canbeset=[]
        self.canbegotten=[]
        self.interface=interface
 
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

######## CHANGE THIS ########################   
            
    def syncout(self):
        commands = self.interface.get_incoming_requests()
        for command in commands:
            if command.action == "command_data":
                inputdata = command.params
                self.t=inputdata['t']
                self.endsignal=inputdata['endsignal']
                self.paused=inputdata['paused']
                self.records=inputdata['records']
                # control_interface.reply_to_command(
                #     request_id=command.request_id, payload=random_int
                # )
                # import pdb; pdb.set_trace()
                print(inputdata)


        # resp = requests.get(sidecarsatelite_url)
        # if resp.status_code==200:
        #     try:
        #         inputdata = json.loads(resp.json()['content'])
        #         self.t=inputdata['t']
        #         self.endsignal=inputdata['endsignal']
        #         self.paused=inputdata['paused']
        #         self.records=inputdata['records']
        #     except:
        #         print('problem reading from ' + sidecarsatelite_url)
            
    def syncin(self):
        outputdata={'instructions':self.instructions}
        self.interface.request_without_reply(
            "command_generic", params=outputdata
        )
        print("sent message")

#         outputdata={'instructions':self.instructions}
#         with LockFile(insyncpath):
#             with open(insyncpath,"w") as output1:     
#                 json.dump(outputdata, output1, sort_keys=True)

######## CHANGE THIS ########################   


class TSolverSideCarSatelite(SideCarSatelite):
    def __init__(self, interface):
        SideCarSatelite.__init__(self, interface)
        self.canbeset=self.can_be_set()
        self.canbegotten=self.can_be_gotten()
        
    def can_be_set(self): # controllable parameters of the model
        return ['Tsource', 'SARsource', 'k', 'sourcescale', 'tend']; 
    
    def can_be_gotten(self): # observables of the model (similar to sensor)
        return ['Tpoint', 'Tvol'];



class TsolverSidecarThread(threading.Thread): 
    def __init__(self):
        threading.Thread.__init__(self)
        self.myTSolverSideCar=TSolverSideCar()
        self.name='TsolverSidecar'
        self.stop=False
    def run(self):
        print("Starting ",self.name)
        while not self.stop:
            time.sleep(0.1)
        print("Exiting ",self.name)




class TsolverSidecarSateliteThread(threading.Thread): 
    def __init__(self, interface):
        threading.Thread.__init__(self)
        self.myTSolverSideCarSatelite=TSolverSideCarSatelite(interface)
        self.name='TsolverSidecarSatelite'
        self.stop=False
    def run(self):
        print("Starting ",self.name)
        while not self.stop:
            time.sleep(0.1)
        print("Exiting ",self.name)



class ControllerThread(threading.Thread): 
    def __init__(self,tweakparam_key, initval, regulationparam_key, regulationparam_otherparams, setpoint, iteration_time, KP, KI, KD, controlled):
        threading.Thread.__init__(self)
        self.myController=Controller(tweakparam_key, initval, regulationparam_key, regulationparam_otherparams, setpoint, iteration_time, KP, KI, KD, controlled)
        self.name='Controller'
    def run(self):
        print("Starting ",self.name)
        self.myController.controllermain()
        print("Exiting ",self.name)
