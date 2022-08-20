#from operator import truediv
#from matplotlib.pyplot import close
from pydantic import BaseModel, Field
from ROAR.control_module.controller import Controller
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle

from ROAR.utilities_module.data_structures_models import Transform, Location
from collections import deque
import numpy as np
import math
import logging
from ROAR.agent_module.agent import Agent
from typing import Tuple
import json
from pathlib import Path

class PIDVController(Controller):
    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_speed = self.agent.agent_settings.max_speed
        throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))
        
        # member variables



        self.old_wideturn=0
        self.old_narrowturn=0
#        self.bigturn=0
        self.old_bigbump=0
        self.old_pitch = 0
#        self.smallbump=0
#        self.pitch_byp=0
        self.latconfig=self.config["latitudinal_controller"]
        self._error_buffer = deque(maxlen=10)
        self._dt = 0.03


        self.logger = logging.getLogger(__name__)

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform, **kwargs) -> VehicleControl:

        
		# statistically optimize the PID to achieve faster speed
        # throttle, brake = self.optimize_speed(next_waypoint,steering, error, wide_error, sharp_error)

	

	
#    def optimize_speed(self,next_waypoint,steering, error, wide_error, sharp_error):
	
	
        # straight regions
        # winding regions
        # bumpy regions
        # corner regions		
	
	
	    # bump_threshold
		# close_turn_threshold
		# far_turn_threshold
		# steering_threshold
		#

        current_speed = Vehicle.get_speed(self.agent.vehicle)
#        current_locationx = self.agent.vehicle.transform.location.x   
#        current_locationz = self.agent.vehicle.transform.location.z	
        current_pitch = float(next_waypoint.record().split(",")[4])		
        current_bumpiness= current_pitch - self.old_pitch
        self.old_pitch = current_pitch      



        # calculate a vector that represent where you are going
        v_begin = self.agent.vehicle.transform.location.to_array()
        direction_vector = np.array([-np.sin(np.deg2rad(self.agent.vehicle.transform.rotation.yaw)),
                                     0,
                                     -np.cos(np.deg2rad(self.agent.vehicle.transform.rotation.yaw))])
        v_end = v_begin + direction_vector

        v_vec = np.array([(v_end[0] - v_begin[0]), 0, (v_end[2] - v_begin[2])])
        
        # calculate error projection
        w_vec = np.array(
            [
                next_waypoint.location.x - v_begin[0],
                0,
                next_waypoint.location.z - v_begin[2],
            ]
        )

        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        _cross = np.cross(v_vec_normed, w_vec_normed)


        w_vec = np.array(
            [
                close_waypoint.location.x - v_begin[0],
                0,
                close_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        wide_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) 

 
        w_vec = np.array(
            [
                far_waypoint.location.x - v_begin[0],
                0,
                far_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        sharp_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) 

        if _cross[1] > 0:
            error *= -1
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0



        k_p, k_d, k_i = 1, 0, 0
        for speed_upper_bound, kvalues in self.latconfig.items():
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
				
        steering = float(
            np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
        )
		
        # error = abs(round(error, 3))
        # wide_error = abs(round(wide_error, 3))
        # sharp_error = abs(round(sharp_error, 3))		
        error = abs(error)
        wide_error = abs(wide_error)
        sharp_error = abs(sharp_error)
        speed_range_mid = 70
        speed_range_high = 90
        bumpiness_high =-2.3
        bumpiness_low =-0.35
        narrowturn_thr =0.6
        wideturn_thr =0.05
        steering_thr =0.3
        bumpy_region_zboundary = 3000
        hilly_region_xboundary = 2800
		
        throttle = 1
        brake = 0
        

    		

#        if current_locationz<bumpy_region_zboundary and current_locationx>hilly_region_xboundary:
#        if True:
		#very bumpy winding hilly areas 
            #print ("1st region")  
        if current_speed <= speed_range_mid : #low speed
            throttle = 1
            brake = 0

            self.old_narrowturn=0
            self.old_wideturn=0            	
            self.old_bigbump=0
        elif current_speed <= speed_range_high : # midium speed
            if current_bumpiness < bumpiness_high and current_speed > 75:
                throttle =-1+self.old_bigbump*0.3
                brake =1-self.old_bigbump*0.3
                self.old_bigbump+=1	
                print("BIGslope: "+str(round(current_bumpiness, 3))+"\t current_speed: "+str(current_speed)+"\t throttle: "+str(throttle)+"\t next_watpoint: ",next_waypoint.record())
            elif abs(steering) > steering_thr : # steering control
                throttle = 0.3
                brake = 0
                print("hardsteering: "+str(steering)+"\t current_speed: "+str(current_speed)+"\t throttle: "+str(throttle)+"\t next_watpoint: ",next_waypoint.record())
				

                self.old_narrowturn=0
                self.old_wideturn=0            	
                self.old_bigbump=0
            else:	
                throttle = 1
                brake = 0						
                self.old_wideturn=0        	
                self.old_narrowturn=0
                self.old_bigbump=0				
        else: # high speed
            if current_bumpiness < bumpiness_high :
                throttle =-1+self.old_bigbump*0.1
                brake =1-self.old_bigbump*0.1
                self.old_bigbump+=1		
                print("BIGslope: "+str(round(current_bumpiness, 3))+"\t current_speed: "+str(current_speed)+"\t throttle: "+str(throttle)+"\t next_watpoint: ",next_waypoint.record())
		
            elif sharp_error > narrowturn_thr and current_speed > 100:
#                throttle = -0.3
 #               brake = 0.7   
                throttle =-0.3+self.old_narrowturn*0.0
                brake =0.7-self.old_narrowturn*0.0
                self.old_narrowturn+=1
                print("narrowturn: "+str(sharp_error)+"\t current_speed: "+str(current_speed)+"\t throttle: "+str(throttle)+"\t next_watpoint: ",next_waypoint.record())
					
#                self.old_bigbump=0
            elif current_bumpiness < bumpiness_low:
                throttle = 0
                brake = 0  
                print("slope: "+str(round(current_bumpiness, 3))+"\t current_speed: "+str(current_speed)+"\t throttle: "+str(throttle)+"\t next_watpoint: ",next_waypoint.record())
        	
#                self.old_bigbump=0
            elif abs(steering) > steering_thr : # steering control
                throttle = 0.3
                brake = 0	
                print("hardsteering: "+str(steering)+"\t current_speed: "+str(current_speed)+"\t throttle: "+str(throttle)+"\t next_watpoint: ",next_waypoint.record())
			
        	
#                self.old_bigbump=0
            elif wide_error > wideturn_thr and current_speed > 95: # wide turn
                throttle = max(0.2, 1 - 6.6*pow(wide_error + current_speed*0.0015, 3))
#                throttle = max(0.37, 1 - 5*(wide_error + current_speed*0.0015))
                brake = 0
                self.old_wideturn+=1
                print("wide_error: "+str(wide_error)+"\t current_speed: "+str(current_speed)+"\t throttle: "+str(throttle)+"\t next_watpoint: ",next_waypoint.record())
	        	
#                self.old_bigbump=0
            else:	
                throttle = 1
                brake = 0						
                self.old_wideturn=0        	
                self.old_narrowturn=0
                self.old_bigbump=0

        return VehicleControl(throttle=throttle, steering=steering, brake=brake)

