# -*- coding: utf-8 -*-
"""
Created on Wed Mar 16 09:14:39 2022

@author: Allan
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from kalmanFilter import kalmanFilter
from helperMethods import helperMethods

#gravity constant
Gravity_const=9.80665

#use json file for data
file_name='pos_final.json'
with open(file_name) as data_file:
    data=json.load(data_file)
    #get initial data
    initial_data=data[0]
    
    #define std deviation for process state vector
    latlonStdDev=2.0
    altStdDev=3.518522417151836
    accEastStdDev=Gravity_const*0.033436506994600976
    accNorthStdDev=Gravity_const*0.05355371135598354
    accUpStdDev=Gravity_const*0.2088683796078286
    
    #create object for helper class
    helperObj=helperMethods()
    
    #create object for kalman filter
    objEast=kalmanFilter(helperObj.lonToMtrs(initial_data['gps_lon']),\
                         initial_data['vel_east'],latlonStdDev,accEastStdDev,\
                         initial_data['timestamp'])
    objNorth=kalmanFilter(helperObj.latToMtrs(initial_data['gps_lat']),\
                          initial_data['vel_north'],latlonStdDev,accNorthStdDev,\
                          initial_data['timestamp'])
    objUp=kalmanFilter(initial_data['gps_alt'],initial_data['vel_down']*-1.0,\
                       altStdDev,accUpStdDev,initial_data['timestamp'])
    

#lists to plot predicted lat & lon
Lat_pts=[]
Lon_pts=[]

#lists to plot original data
org_Lat=[]
org_Lon=[]

for i in range(1,len(data)):
    curr_data=data[i]
    
    #call priori objects
    objEast.predict(curr_data['abs_east_acc']*Gravity_const,curr_data['timestamp'])
    objNorth.predict(curr_data['abs_north_acc']*Gravity_const,curr_data['timestamp'])
    objUp.predict(curr_data['abs_up_acc']*Gravity_const,curr_data['timestamp'])
    
    #if GPS data is not zero
    if(curr_data['gps_lat']!=0.0):
        defPosErr=0.0
        
        vEast=curr_data['vel_east']
        longitude=objEast.lonToMtrs(curr_data['gps_lon'])
        objEast.updata(longitude,vEast,defPosErr,curr_data['vel_error'])
        
        vNorth=curr_data['vel_north']
        latitude=objNorth.latToMtrs(curr_data['gps_lat'])
        objNorth.update(latitude,vNorth,defPosErr,curr_data['vel_error'])

        vUp=curr_data['vel_down']*-1.0
        altitude=curr_data['gps_alt']
        objUp.update(altitude,vUp,curr_data['altitude_error'],curr_data['vel_error'])
        
        #move original gps data to lists
        org_Lat.append(curr_data['gps_lat'])
        org_Lon.append(curr_data['gps_lon'])
    
    #get predicted values
    #get lat,lon,alt in meters
    predicted_LonMtrs=objEast.getPredictedPos()
    predicted_LatMtrs=objNorth.getPredictedPos()
    predicted_Alt=objUp.getPredictedPos()
    
    #convert meters to geopoint
    predicted_Lat,predicted_Lon=helperObj.mtrstoGeopt(predicted_LatMtrs,predicted_LonMtrs)
    
    predicted_VelEast=objEast.getPredictedVel()
    predicted_VelNorth=objNorth.getPredictedVel()
    
    final_predicted_V=np.sqrt(np.power(predicted_VelEast,2)+np.power(predicted_VelNorth,2))
    
    deltaT=curr_data['timestamp']-initial_data['timestamp']
    
    #append predicted Lat,Lon to list
    Lat_pts.append(predicted_Lat)
    Lon_pts.append(predicted_Lon)

plt.subplot(2,1,1)
plt.title('original')
plt.plot(org_Lat,org_Lon)

plt.subplot(2,1,2)
plt.title('title')
plt.plot(Lat_pts,Lon_pts)
