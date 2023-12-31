#!/usr/bin/env python2

import rospy
import utm
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
import tf
import time
import math
import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from gps_common.msg import GPSFix
from std_msgs.msg import UInt16




##############################################################################################
#      Rviz
##############################################################################################

class Marker_rviz():

	def __init__(self,name,position=(0,0,0), angles=(0,0,0), scale=(1,1,1),m_type=1,color=(1.0,1.0,1.0)):
		"""
		name : string, name of the marker
		type : int, type of the marker
		position : tuple, (x,y,z)
		angles : tuple, (roll,pitch,yaw)
		scale : tuple, (x,y,z)
		"""
		self.marker = Marker()
		self.marker.header.frame_id = name
		self.marker.header.stamp = rospy.get_rostime()
		self.marker.ns = 'ns_'+name
		self.marker.id = 0
		self.marker.action = 0
		self.marker.pose.position.x = position[0]
		self.marker.pose.position.y = position[1]
		self.marker.pose.position.z = position[2]
		q = quaternion_from_euler(angles[0],angles[1],angles[2])
		self.marker.pose.orientation.x = q[0]
		self.marker.pose.orientation.y = q[1]
		self.marker.pose.orientation.z = q[2]
		self.marker.pose.orientation.w = q[3]
		self.marker.scale.x = scale[0]
		self.marker.scale.y = scale[1]
		self.marker.scale.z = scale[2]
		self.marker.color.r = color[0]
		self.marker.color.g = color[1]
		self.marker.color.b = color[2]
		self.marker.color.a = 1.0

		self.marker.type = m_type
		if m_type == 10:
			self.marker.mesh_resource = "package://wrsc_plymouth_jegat/meshs/"+name+".STL"

		self.pub_send_marker = rospy.Publisher('forrviz_send_'+name, Marker, queue_size=10)

	def publish(self):
		self.marker.header.stamp = rospy.get_rostime()
		self.pub_send_marker.publish(self.marker)

	# if m_type == 4
	def set_points(self,set_p1,set_p2):
		global lat_lon_origin
		p1 = Point()
		p1.x,p1.y = set_p1 #-75,40
		self.marker.points.append(p1)
		p2 = Point()
		p2.x,p2.y = set_p2 # 175,-40
		self.marker.points.append(p2)

##############################################################################################
#      ROS
##############################################################################################

def sub_euler_angles(data):
	global yaw,pitch,roll
	yaw = data.x
	pitch = data.y
	roll = data.z

def sub_xy(data):
	global x, y, delta_s
#	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
#	x = data.x
#	y = data.y
#	delta_s = data.z

def sub_wind_direction(data):
	global psi
	psi = data.data

def sub_wind_force(data):
	global awind
	awind = data.data

def sub_u_rudder(data):
	global dr
	#rospy.loginfo("U_deltar : %s, U_deltamax : %s, Q : %s",data.x, data.y, data.z)
	dr = data.data

previous_mode="mode10"
mode = "mode10"

def sub_u_sail(data):

    global delta_s,previous_mode,mode,delta_commande

    theta_1 = 135
    theta_2 = 225
    delta_theta = 5
    theta = abs((math.fmod((180/np.pi)*(np.pi+psi-yaw),360)))
    if theta<theta_1-delta_theta and previous_mode == "mode90":
	delta_s = np.pi + np.deg2rad(10) + (psi - yaw)
	mode ="mode10"
    elif theta > theta_2+delta_theta and previous_mode == "mode90":
	delta_s = np.pi + np.deg2rad(10) + (psi - yaw)
	mode ="mode10"
    elif theta_1+delta_theta<theta < theta_2-delta_theta and previous_mode == "mode10":
	delta_s= np.pi/2 + (psi-yaw)
	mode="mode90"
    elif theta_2-delta_theta>theta>theta_1+delta_theta and previous_mode == "mode10":
	delta_s= np.pi/2 + (psi-yaw)
	mode="mode90"
    elif theta_1-delta_theta<theta<theta_2+delta_theta and previous_mode == "mode90":
	delta_s= np.pi/2 + (psi-yaw)
	mode="mode90"
    elif ((theta_1+delta_theta>theta) or (theta >theta_2-delta_theta)) and previous_mode == "mode10":
	delta_s = np.pi + np.deg2rad(10) + (psi - yaw)
	mode ="mode10"
    previous_mode=mode
    delta_commande = delta_s
    return delta_s
"""
def sub_u_sail(data):
	global delta_s
	if 135<abs((math.fmod((180/np.pi)*(np.pi+psi-yaw),360)))<225:
		delta_s= np.pi/2 + (psi-yaw)
	else :
		delta_s = np.pi + (np.pi/180)*10 + (psi-yaw) # not the exat definition but still ok for display
	print abs((math.fmod((180/np.pi)*(np.pi+psi-yaw),360)))
        return delta_s
"""
def sub_gps_origin(data): # Vector3
	global lat_lon_origin
	lat_lon_origin[0] = [data.x, data.y]
	lat_lon_origin[1] = utm.from_latlon(data.x, data.y)

def sub_gps(data): # Vector3
	global x, y, v
	res = utm.from_latlon(data.latitude, data.longitude)
	v = data.speed

	x = -(lat_lon_origin[1][1]-res[1])
	y = lat_lon_origin[1][0]-res[0]

def sub_lines_to_follow(data): # Quaternion
	global pline

	res = utm.from_latlon(data.x, data.y)
	pline[0][0] = -(lat_lon_origin[1][1]-res[1])
	pline[0][1] = (lat_lon_origin[1][0]-res[0])

	res = utm.from_latlon(data.z, data.w)
	pline[1][0] = -(lat_lon_origin[1][1]-res[1])
	pline[1][1] = (lat_lon_origin[1][0]-res[0])





##############################################################################################
#      Main
##############################################################################################

if __name__ == "__main__":
	yaw,pitch,roll = 0,0,0
	pline = [[0,0],[1,1]]
	awind, psi = 1,-np.pi
	delta_s, dr = 0,0
	delta_commande = delta_s
	x, y, v = 0, 0, 0
	lat_lon_origin = [[],[]]
	delta_flap=(np.pi/180)*15
	rospy.init_node('rviz_displayer_line_following')
	rospy.Subscriber("simu_send_theta", Vector3, sub_euler_angles)
	rospy.Subscriber("simu_send_xy", Point, sub_xy)
	rospy.Subscriber("simu_send_wind_direction", Float32, sub_wind_direction)
	rospy.Subscriber("simu_send_wind_force", Float32, sub_wind_force)
	rospy.Subscriber("control_send_u_rudder", Float32, sub_u_rudder)
	rospy.Subscriber("control_send_u_sail", Float32, sub_u_sail)
	rospy.Subscriber("launch_send_gps_origin", Vector3, sub_gps_origin)
	rospy.Subscriber("simu_send_gps", GPSFix, sub_gps)
	rospy.Subscriber("control_send_lines_to_follow", Quaternion, sub_lines_to_follow)
	

	marker_boat   = Marker_rviz("boat",(-0.5,-0.24,-0.2),(np.pi/2, 0, np.pi/2),(0.0002,0.0002,0.0002),10,(0.9,0.08,0))
	marker_rudder = Marker_rviz("rudder",(0.15,0,-0.2),(np.pi/2, 0, -np.pi/2),(0.0004,0.0004,0.0004),10)
	marker_sail   = Marker_rviz("sail",(0.55,0,0),(np.pi/2, 0, -np.pi/2),(0.0002,0.0002,0.0002),10,(0.8,0.64,0.64))
	marker_wind   = Marker_rviz("wind",(0,0,0),(0, 0, 0),(awind+0.01,0.1,0.1),0)
	marker_line   = Marker_rviz("line",(0,0,0),(0, 0, 0),(.2,0,0),4,(0,1,0))
#Zephyr
	marker_coque   = Marker_rviz("coque",(0,0,0),(np.pi/2, 0, np.pi/2),(0.2,0.2,0.2),10,(0.6,0.6,0.6))
	marker_safran = Marker_rviz("safran",(0,0,0),(np.pi/2, 0, -np.pi/2),(0.2,0.2,0.2),10,(1,1,0))
	marker_voile1   = Marker_rviz("voile1",(0,0,0),(np.pi/2, 0, -np.pi/2),(0.2,0.2,0.2),10,(1,1,1))
	marker_voile2   = Marker_rviz("voile2",(0,0,0),(np.pi/2, 0, -np.pi/2),(0.2,0.2,0.2),10,(1,1,1))
	marker_voile3   = Marker_rviz("voile3",(0,0,0),(np.pi/2, 0, -np.pi/2),(0.2,0.2,0.2),10,(1,1,1))
	marker_voile4   = Marker_rviz("voile4",(0,0,0),(np.pi/2, 0, -np.pi/2),(0.2,0.2,0.2),10,(1,1,1))
	rate = rospy.Rate(1) # mettre 100hz pour avoir une simulation smooth


	
	pub_commande_verin = rospy.Publisher('commande_verin', UInt16, queue_size=10)

	pub_delta_s = rospy.Publisher('delta_sail_1', Float32, queue_size=10)
	


while not rospy.is_shutdown():
		marker_boat.publish()
		marker_rudder.publish()
		marker_sail.publish()
		marker_wind.publish()
		marker_line.publish()
#Zephyr
		marker_coque.publish()
		marker_safran.publish()
		marker_voile1.publish()
		marker_voile2.publish()
		marker_voile3.publish()
		marker_voile4.publish()
		pub_delta_s.publish(delta_s*180/np.pi)
		if delta_commande < 0:
    			delta_commande = 2*math.pi + delta_commande
		elif delta_commande > 2*math.pi:
   			delta_commande = delta_commande % (2*math.pi)
		pub_commande_verin.publish(int(delta_commande * (4096 / (2 * np.pi))))
		

		rate.sleep()
		
#		br_boat = tf.TransformBroadcaster()
#		br_boat.sendTransform((x, y, 0.0),quaternion_from_euler(roll,pitch,yaw),rospy.Time.now(),"boat","map")
#		br_rudder = tf.TransformBroadcaster()
#		br_rudder.sendTransform((-0.5,0,0),(quaternion_from_euler(0,0,np.pi+dr)),rospy.Time.now(),"rudder","boat")
#		br_sail = tf.TransformBroadcaster()
#		br_sail.sendTransform((0.1,0,0.3),(quaternion_from_euler(0,0,np.pi+delta_s)),rospy.Time.now(),"sail","boat")
		br_wind = tf.TransformBroadcaster()
		br_wind.sendTransform((x+2,y,0),(quaternion_from_euler(0,0,psi)),rospy.Time.now(),"wind","map")
		br_line = tf.TransformBroadcaster()
		br_line.sendTransform((0,0,0),(quaternion_from_euler(0,0,0)),rospy.Time.now(),"line","map")
		marker_line.set_points(pline[0],pline[1])
			
#Zephyr		
		br_coque = tf.TransformBroadcaster()
		br_coque.sendTransform((x, y, 0.0),quaternion_from_euler(roll,pitch,yaw),rospy.Time.now(),"coque","map")
		br_safran = tf.TransformBroadcaster()
		br_safran.sendTransform((-0.38,0,-0.095),(quaternion_from_euler(0,0,dr)),rospy.Time.now(),"safran","coque")
		
		if mode == "mode10":
			br_voile1 = tf.TransformBroadcaster()
			br_voile1.sendTransform((0.2315,0,0.01),(quaternion_from_euler(0,0,delta_s)),rospy.Time.now(),"voile1","coque")
			br_voile2 = tf.TransformBroadcaster()
			br_voile2.sendTransform((-0.04,0,0.095),
	(quaternion_from_euler(0,0,delta_flap)),rospy.Time.now(),"voile2","voile1")
			br_voile3 = tf.TransformBroadcaster()
			br_voile3.sendTransform((-0.135,0,0.01),(quaternion_from_euler(0,0,delta_s)),rospy.Time.now(),"voile3","coque")
			br_voile4 = tf.TransformBroadcaster()
			br_voile4.sendTransform((-0.04,0,0.095),
	(quaternion_from_euler(0,0,delta_flap)),rospy.Time.now(),"voile4","voile3")

		else :
			br_voile1 = tf.TransformBroadcaster()
			br_voile1.sendTransform((0.2315,0,0.01),(quaternion_from_euler(0,0,delta_s)),rospy.Time.now(),"voile1","coque")
			br_voile2 = tf.TransformBroadcaster()
			br_voile2.sendTransform((-0.04,0,0.095),
	(quaternion_from_euler(0,0,0)),rospy.Time.now(),"voile2","voile1")
			br_voile3 = tf.TransformBroadcaster()
			br_voile3.sendTransform((-0.135,0,0.01),(quaternion_from_euler(0,0,delta_s+np.pi)),rospy.Time.now(),"voile3","coque")
			br_voile4 = tf.TransformBroadcaster()
			br_voile4.sendTransform((-0.04,0,0.095),
	(quaternion_from_euler(0,0,0)),rospy.Time.now(),"voile4","voile3")

