

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
                print(self.t)

    def syncin(self):
        outputdata={'instructions':self.instructions}
        self.interface.request_without_reply(
            "command_generic", params=outputdata
        )


######## CHANGE THIS ########################   

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
        commands = self.interface.get_incoming_requests()
        for command in commands:
            print("received " + str(command))
            if command.action == "command_generic":
                inputdata = command.params
                print(str(inputdata))

        if inputdata != None:
            self.instructions=inputdata['instructions']
            self.executeInstructions()
            print("Successfully executed " + str(inputdata))
        ##### CHANGE #########    
            
    def syncout(self):
        ##### CHANGE #########
        outputdata={'t':self.t, 'endsignal':self.endsignal, 'paused':self.paused, 'records':self.records} # start?
        self.interface.request_without_reply(
            "command_data", params=outputdata
        )            

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
