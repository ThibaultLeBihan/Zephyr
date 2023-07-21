#!/usr/bin/env python2

import matplotlib.pyplot as plt
import numpy as np
from numpy import cos,sin
import utm
import math
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from gps_common.msg import GPSFix

lat_lon_origin = [[],[]]

##############################################################################################
#      ROS
##############################################################################################

def sub_u_rudder(data):
    global u
    u[0,0] = data.data

def sub_u_sail(data):
    global u
    u[1,0] = data.data

def sub_gps_origin(data):
    global lat_lon_origin
    lat_lon_origin[0] = [data.x, data.y]
    lat_lon_origin[1] = utm.from_latlon(data.x, data.y)

##############################################################################################
#      Euler integration
##############################################################################################
previous_mode="mode10"
mode = "mode10"
def sail_position(x,u):
    x,u=x.flatten(),u.flatten()
    theta=x[2]; v=x[3]; w=x[4]; delta_r=u[0];
    w_ap = np.array([[awind*cos(psi-theta) - v],[awind*sin(psi-theta)]])
    psi_ap = angle(w_ap)
    global delta_s,previous_mode,mode

    theta_1 = 135
    theta_2 = 225
    delta_theta = 5
    theta = abs((math.fmod((180/np.pi)*(np.pi+psi_ap),360)))
    if theta<theta_1-delta_theta and previous_mode == "mode90":
	delta_s = np.pi + np.deg2rad(10) + psi_ap
	mode ="mode10"
    elif theta > theta_2+delta_theta and previous_mode == "mode90":
	delta_s = np.pi + np.deg2rad(10) + psi_ap
	mode ="mode10"
    elif theta_1+delta_theta<theta < theta_2-delta_theta and previous_mode == "mode10":
	delta_s= np.pi/2 + psi_ap
	mode="mode90"
    elif theta_2-delta_theta>theta>theta_1+delta_theta and previous_mode == "mode10":
	delta_s= np.pi/2 + psi_ap
	mode="mode90"
    elif theta_1-delta_theta<theta<theta_2+delta_theta and previous_mode == "mode90":
	delta_s= np.pi/2 + psi_ap
	mode="mode90"
    elif ((theta_1+delta_theta>theta) or (theta >theta_2-delta_theta)) and previous_mode == "mode10":
	delta_s = np.pi + np.deg2rad(10) + psi_ap
	mode ="mode10"
    previous_mode=mode
    return delta_s

def angle(x):
    x=x.flatten()
    return np.arctan2(x[1],x[0])


def f(x,u):
    x,u=x.flatten(),u.flatten()
    theta=x[2]; v=x[3]; w=x[4]; delta_r=u[0];
    w_ap = np.array([[awind*cos(psi-theta) - v],[awind*sin(psi-theta)]])
    psi_ap = angle(w_ap)
    a_ap=np.linalg.norm(w_ap)
    fr = p4*v*sin(delta_r)
    if mode == "mode10":
	fs=0.5*a_ap*a_ap*1.293*0.8*1.4*math.sqrt(p3*p3+p11*p11)
	thrust = fs * sin (psi_ap-delta_s)
	dw=(fs*(p5-p6*cos(delta_s)) + fs*(p5-p10*cos(delta_s)) - p7*fr*cos(delta_r) - p2*w*v)/p9
    elif  mode == "mode90":
	fs = 0.5*a_ap*a_ap*1.293*0.8*1.4*p12
	thrust = fs*cos(psi_ap)
	dw=(-fs*(p5-p6*cos(delta_s)) + fs*(p5-p10*cos(delta_s)) - p7*fr*cos(delta_r) - p2*w*v)/p9
    
    dx=v*cos(theta) + p0*awind*cos(psi)
    dy=v*sin(theta) + p0*awind*sin(psi)
    dv=(fs-fr*sin(delta_r)-p1*v**2)/p8 
    #print("thrust :",thrust)
    #return(thrust)
    xdot=np.array([[dx],[dy],[w],[dv],[dw]])
    return xdot,delta_s
"""
Pour le comportement de l'autre bateau :
def f(x,u):
    x,u=x.flatten(),u.flatten()
    theta=x[2]; v=x[3]; w=x[4]; delta_r=u[0]; delta_smax=u[1];
    w_ap = np.array([[awind*cos(psi-theta) - v],[awind*sin(psi-theta)]])
    psi_ap = angle(w_ap)
    a_ap=np.linalg.norm(w_ap)
    sigma = cos(psi_ap) + cos(delta_smax)
    if sigma < 0 :
        delta_s = pi + psi_ap
    else :
        delta_s = -np.sign(sin(psi_ap))*delta_smax
    fr = p4*v*sin(delta_r)
    fs = p3*a_ap* sin(delta_s - psi_ap)
    dx=v*cos(theta) + p0*awind*cos(psi)
    dy=v*sin(theta) + p0*awind*sin(psi)
    dv=(fs*sin(delta_s)-fr*sin(delta_r)-p1*v**2)/p8
    dw=(fs*(p5-p6*cos(delta_s)) - p7*fr*cos(delta_r) - p2*w*v)/p9
    xdot=np.array([[dx],[dy],[w],[dv],[dw]])
    return xdot,delta_s

##############################################################################################
#      Main
##############################################################################################

if __name__ == '__main__':

    # --- Boat variables --- #

    p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.03,40,6000,200,1500,0.5,0.5,2,300,400#0.1,0.1,0.5,3,1,0,0,0.5,10,1 #0.1,1,6000,1000,2000,1,1,2,300,10000
"""
##############################################################################################
#      Main
##############################################################################################

if __name__ == '__main__':

    # --- Boat variables --- #
    
    p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12 = 0.03,400,6000,60,1500,0.2315,0.5,2,300,400,0.135,4,230#0.1,0.1,0.5,3,1,0,0,0.5,10,1 #0.1,1,6000,1000,2000,1,1,2,300,10000

    x = np.array([[0,0,0,1,0]]).T #x=(x,y,theta,v,w)
    dt = 0.01
    pi = np.pi
    awind,psi = 2, 0
    u = [0,0]

    # --- ROS -------------- #

    pub_send_theta           = rospy.Publisher('simu_send_theta', Vector3, queue_size=10)
    pub_send_xy              = rospy.Publisher('simu_send_xy', Vector3, queue_size=10)
    pub_send_wind_direction  = rospy.Publisher('simu_send_wind_direction', Float32, queue_size=10)
    pub_send_wind_force      = rospy.Publisher('simu_send_wind_force', Float32, queue_size=10)
    pub_send_gps             = rospy.Publisher('simu_send_gps', GPSFix, queue_size=10)
    theta_msg          = Vector3()
    xy_msg             = Vector3()
    wind_direction_msg = Float32()
    wind_force_msg     = Float32()
    gps_msg            = GPSFix()
    rospy.Subscriber("control_send_u_rudder", Float32, sub_u_rudder)
    rospy.Subscriber("control_send_u_sail", Float32, sub_u_sail)
    rospy.Subscriber("launch_send_gps_origin", Vector3, sub_gps_origin)
    node_name = 'simu_boat'
    rospy.init_node(node_name)
    rate = rospy.Rate(1/dt) # 10hz

    # --- Main ------------- #
    u = np.array([[0],[1.0]])
    while lat_lon_origin == [[],[]]:
        rospy.sleep(0.5)
        rospy.loginfo("[{}] Waiting GPS origin".format(node_name))
    rospy.loginfo("[{}] Got GPS origin {}".format(node_name,lat_lon_origin))

    while not rospy.is_shutdown():
	delta_s=sail_position(x,u) 
        xdot,delta_s=f(x,u)
	#thrust=f2(x,u)
        x = x + dt*xdot
        #rospy.loginfo("[{}] x : {}, y:{}".format(node_name,x[0,0],x[1,0]))
        #rospy.loginfo("[{}] theta : {}".format(node_name,x[2,0]))
        theta_msg.x = x[2,0] # heading
        xy_msg.x = x[0,0]    # x pos
        xy_msg.y = x[1,0]    # y pos
        xy_msg.z = delta_s
        wind_direction_msg.data = psi   # wind direction
        wind_force_msg.data = awind     # wind speed
        utm_to_latlon = utm.to_latlon(lat_lon_origin[1][0]-x[1,0],lat_lon_origin[1][1]+x[0,0], lat_lon_origin[1][2],lat_lon_origin[1][3])
        gps_msg.latitude = utm_to_latlon[0]
        gps_msg.longitude = utm_to_latlon[1]
        gps_msg.speed = x[3,0] #speed

        pub_send_theta.publish(theta_msg)
        pub_send_xy.publish(xy_msg)
        pub_send_wind_direction.publish(wind_direction_msg)
        pub_send_wind_force.publish(wind_force_msg)
        pub_send_gps.publish(gps_msg)

        rate.sleep()
