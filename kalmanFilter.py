# -*- coding: utf-8 -*-
"""
Created on Wed Mar 16 13:41:15 2022

@author: Allan
"""

import numpy as np
from helperMethods import helperMethods

class kalmanFilter(helperMethods):
    '''
    Fuse IMU and GPS measurements
    '''
    
    def __init__(self,initPos,initVel,posStdDev,accStdDev,currTime):
        helperMethods.__init__(self)
        #from the received arguments, set values for below parameters
        #current state
        self.X=np.array([[np.float64(initPos)],[np.float64(initVel)]])
        #Identity matrix
        self.I=np.identity(2)
        #Initial guess for covariance
        self.P=np.identity(2)
        #Transformation matrix for input u(t)
        self.H=np.identity(2)
        #Process(IMU) variance
        self.Q=np.array([[accStdDev*accStdDev,0],[0,accStdDev*accStdDev]])
        #Measurement (GPS) variance
        self.R=np.array([[posStdDev*posStdDev,0],[0,posStdDev*posStdDev]])
        #current time
        self.currentTime=currTime
        
        #main functions
    def predict(self,accThisAxis,timeNow):
        '''
        To Predict current state and compute P matrix (takes only acceleration data)
        '''
        deltaT=timeNow-self.currentTime
        self.B=np.array([[0.5*deltaT*deltaT],[deltaT]])
        self.A=np.array([[1.0,deltaT],[0.0,1.0]])
        self.u=np.array([[accThisAxis]])
           
        self.X=np.add(np.matmul(self.A,self.X),np.matmul(self.B,self.u)) #priori estimate
        self.P=np.add(np.matmul(np.matmul(self.A,self.P),np.transpose(self.A)),self.Q) 
            
        self.currStateTime=timeNow
        #update of Kalman filter
    def update(self,pos,velThisAxis,posError,velError):
        '''
        Updates the predicted state using GPS measurements
        '''
        self.z=np.array([[pos],[velThisAxis]])
        if(not posError):
            self.R[0,0]=posError*posError
        else:
            self.R[1,1]=velError*velError
        y=np.subtract(self.z,self.X) #residue
        s=np.add(self.P,self.R)
        try:
            sInverse=np.linalg.inv(s)
        except np.linalg.LinAlgError:
            print('matrix not invertible')
            pass
        else:
            K=np.matmul(self.P,sInverse) #Kalman Gain
            self.X=np.add(self.X,np.matmul(K,y)) #posteriori estimate
            self.P=np.matmul(np.subtract(self.I,K),self.P)
        
    def getPredictedpos(self):
        '''
        returns predicted position for that axis
        '''
        return self.X[0,0]
    
    def getPredictedVel(self):
        '''
        returns predicted velocity for that axis
        '''
        return self.X[1,0]
            
        