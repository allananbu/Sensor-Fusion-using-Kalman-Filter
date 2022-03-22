# -*- coding: utf-8 -*-
"""
Created on Wed Mar 16 10:19:49 2022

@author: Allan
"""

import numpy as np

EARTH_RADIUS=6378*1000 #convert to meters


class helperMethods(object):
    ''' 
    contains functions that handles GPS data calculations
    & conversion from Radians to Degrees or computes distance between 
    GPS pts
    '''
    
    def __init__(self):
        pass
    
    
    def latToMtrs(self,latitude):
        '''
        Latitude to meters
        '''
        distance=self.getDistMtrs(latitude,0.0,0.0,0.0)
        
        if(distance<0):
            distance*=-1
        return distance
    
    def lonToMtrs(self,longitude):
        '''
        Longitude to meters
        '''
        distance=self.getDistMtrs(0.0,longitude,0.0,0.0)
        
        if(distance<0):
            distance*=-1
        return distance
    
    def degToRad(self,LatorLon):
        '''
        Convert Degrees to radians
        '''
        return (LatorLon*np.pi)/180.0
    
    def radToDeg(self,LatorLon):
        '''
        Convert Radians to degrees
        '''
        return (LatorLon*180.0)/np.pi
    
    def getDistMtrs(self,lat_from,lon_from,lat_to,lon_to):
        '''
        Get Distance between two GPS points
        '''
        deltaLon=self.degToRad(lon_to-lon_from)
        deltaLat=self.degToRad(lat_to-lat_from)
        
        #Haversine formula
        a=np.power(np.sin(deltaLat/2.0),2)+\
        np.cos(self.degToRad(lat_from))*np.cos(self.degToRad(lat_to))*\
        np.power(np.sin(deltaLon/2.0),2)
        
        c=2*np.arctan2(np.sqrt(a),np.sqrt(1.0-a))
        
        return EARTH_RADIUS*c
            
    def getPointAhead(self,lat_from,lon_from,distMtrs,azimuth):
        '''
        function for meters to geopoint
        formula-find destination given distance and bearing from starting pt
        φ2 = asin( sin φ1 ⋅ cos δ + cos φ1 ⋅ sin δ ⋅ cos θ )
        λ2 = λ1 + atan2( sin θ ⋅ sin δ ⋅ cos φ1, cos δ − sin φ1 ⋅ sin φ2 )
        where	φ is latitude, λ is longitude, θ is the bearing (clockwise from north), 
        δ is the angular distance d/R; d being the distance travelled, R the earth’s radius
        '''
        radiusFraction=distMtrs/EARTH_RADIUS
        bearing=self.degToRad(azimuth)
        lat1=self.degToRad(lat_from)
        lon1=self.degToRad(lon_from)
        
        lat2_part1=np.sin(lat1)*np.cos(radiusFraction)
        lat2_part2=np.cos(lat1)*np.sin(radiusFraction)*np.cos(bearing)
        
        lat2=np.arcsin(lat2_part1+lat2_part2) #final latitude
        
        lon2_part1=np.sin(bearing)*np.sin(radiusFraction)*(np.cos(lat1))
        lon2_part2=np.cos(radiusFraction)-np.sin(lat1)*np.sin(lat2)
        
        lon2=lon1+np.arctan2(lon2_part1,lon2_part2)
        lon2 = np.mod((lon2 + 3 * np.pi), (2 * np.pi)) - np.pi #why
        
        return self.radToDeg(lat2),self.radToDeg(lon2)
    
    #Convert longitude and latitude separately
    def mtrstoGeopt(self,latAsMtrs,lonAsMtrs):
        '''
        conversion from meters to geopoint
        '''
        lat_tmp,lon_tmp=self.getPointAhead(0.0,0.0,lonAsMtrs,90.0)
        lat_geo,lon_geo=self.getPointAhead(lat_tmp,lon_tmp,latAsMtrs,0.0)
        
        return lat_geo,lon_geo

        
        
        