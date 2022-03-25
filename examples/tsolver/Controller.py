

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

from communication import SideCar
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
        while not self.controlled.endsignal:
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



class TSolverSideCarSatelite(SideCar):
    def __init__(self, interface):
        SideCar.__init__(self, interface, "REQUESTER")
        self.canbeset=self.can_be_set()
        self.canbegotten=self.can_be_gotten()
        
    def can_be_set(self): # controllable parameters of the model
        return ['Tsource', 'SARsource', 'k', 'sourcescale', 'tend']; 
    
    def can_be_gotten(self): # observables of the model (similar to sensor)
        return ['Tpoint', 'Tvol'];


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
