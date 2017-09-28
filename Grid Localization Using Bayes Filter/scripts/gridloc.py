#!/usr/bin/env python
print "Lab 4"
import rosbag
from interactive_markers.interactive_marker_server import *
import roslib
import sys
import rospkg
import math	
import numpy as np
from geometry_msgs.msg import Point
import scipy.ndimage
from visualization_msgs.msg import *
from tf.transformations import euler_from_quaternion
roslib.load_manifest('lab4')
import scipy.stats as sc
rospack=rospkg.RosPack()
path=rospack.get_path('lab4')
path=path+'/data/grid.bag'
bag=rosbag.Bag(path)
import rospy

mov=[]
obs=[]

#Extraction of messages
for topic, msg, t in bag.read_messages(topics=['Movements']):
	mov.append(msg)

for topic, msg, t in bag.read_messages(topics=['Observations']):
	obs.append(msg)

#Refinement of messages
def refine(msg):
	msg=str(msg)
	msg=msg[msg.index('z: '):]
	z1=float(msg[msg.index('z')+3:msg.index('z')+13])
	msg=msg[msg.index('w: '):]
	w1=float(msg[msg.index('w')+3:msg.index('w')+13])
	msg=msg[msg.index('n: '):]
	t=float(msg[msg.index('n')+3:msg.index('n')+9])
	msg=msg[msg.index('z: '):]
	z2=float(msg[msg.index('z')+3:msg.index('z')+13])
	msg=msg[msg.index('w: '):]
	w2=float(msg[msg.index('w')+3:msg.index('w')+13])
		
	return z1,w1,t,z2,w2
def refine2(msg):
	msg=str(msg)
	msg=msg[msg.index('m: '):]
	tag=int(msg[3])
	msg=msg[msg.index('e: '):]
	r=float(msg[msg.index('e')+3:msg.index('e')+8])
	msg=msg[msg.index('z: '):]
	z=float(msg[msg.index('z')+3:msg.index('z')+13])
	msg=msg[msg.index('w: '):]
	w=float(msg[msg.index('w')+3:msg.index('w')+13])

	return tag,r,z,w

#Rotation operations
def rtrop(t,yaw,yaw2):
	index=[]
	for i in range(0,35):
		for j in range (0,35):
			for k in range (0,36):
				if m.grid[i,j,k]>0.0001:
					temp=[i,j,k]
					index.append(temp)

	index2=[]				
	for i in index:
		temp=i
		ang=temp[2]*10+5
		ang=ang*0.0174533           #Conversion from degrees to radians
		ang=ang+(yaw)
		x,y=coord(temp[0],temp[1])
		x=x+(t*100)*math.cos(ang)
		y=y+(t*100)*math.sin(ang)
		ang=ang+yaw2
		x,y=cell(x,y)
		if ang<0:
			ang=2*3.1415+ang    	#taking care of negative angles
		if ang>2*3.1415:
			ang=ang-2*3.1415		#taking care of angles > 360 degrees
		ang=int(ang*57.2958/10)
		temp2=[x,y,ang]
		index2.append(temp2)

	grid2=np.zeros((35,35,36))
	n=len(index2)
	for i in range(0,n):
		l=index[i]				#indices of parent array
		temp=m.grid[l[0],l[1],l[2]]
		 
		m.grid[l[0],l[1],l[2]]=0
		l=index2[i]            #indices of the array to be updated
		grid3=gaussian1(temp,l)
		grid2=grid2+grid3

	m.grid=m.grid+grid2
	
		
#Movement operations
def movop(msg):
	z1,w1,t,z2,w2=refine(msg)
	(roll, pitch, yaw) = euler_from_quaternion([0,0,z1,w1])
	(roll2, pitch2, yaw2) = euler_from_quaternion([0,0,z2,w2])
	rtrop(t,yaw,yaw2)
	
def gaussian1(temp,l):
	p,q,r=l[0],l[1],l[2]
	if 0<p<34 and 0<q<34 and 0<r<35:
		x=np.zeros((3,3,3))
		sigma1=2				#Change sigma1 here
		for i in range(0,3):
			for j in range(0,3):
				for k in range(0,3):
					x[i][j][k]=sc.multivariate_normal.pdf([i,j,k],[1,1,1],[[sigma1,0,0],[0,sigma1,0],[0,0,sigma1]])

		x=temp*x
		grid=np.zeros((35,35,36))
		grid[p-1:p+2,q-1:q+2,r-1:r+2]=x
		return grid

	else:
		grid=np.zeros((35,35,36))
		grid[p,q,r]=temp
		return grid

def gaussian2(temp,l):
	p,q,r=l[0],l[1],l[2]
	if 0<p<34 and 0<q<34 and 0<r<35:
		x=np.zeros((3,3,3))
		sigma2=4				#Change sigma2 here
		for i in range(0,3):
			for j in range(0,3):
				for k in range(0,3):
					x[i][j][k]=sc.multivariate_normal.pdf([i,j,k],[1,1,1],[[sigma2,0,0],[0,sigma2,0],[0,0,sigma2]])

		x=temp*x
		grid=np.zeros((35,35,36))
		grid[p-1:p+2,q-1:q+2,r-1:r+2]=x
		return grid

	else:
		grid=np.zeros((35,35,36))
		grid[p,q,r]=temp
		return grid
		


	
	
#Observation operations
def obsop(msg):
	tag,tr,z,w=refine2(msg)
	(roll, pitch, yaw) = euler_from_quaternion([0,0,z,w])
	bearing=yaw
	#print yaw
	k=m.tags[tag]
	x=k[0]							#co-ordinates of tags
	y=k[1]
	#print x,y
	maxval=np.amax(m.grid)			#max of grid
	count=0
	maxes=[]
	for i in range(0,35):
		for j in range (0,35):
			for k in range (0,36):
				if m.grid[i,j,k]==maxval:
					marray=[i,j,k]
					maxes.append(marray)
					count=count+1
	
	
	marray=maxes[0]
	i,j,c=marray[0],marray[1],marray[2]
	#print i,j,c
	x2,y2=coord(i,j)
	ang=0.0
	ang=c*10+5
	ang=ang*0.0174533
	ang=bearing+ang
	if ang<0:
		ang=2*3.1415+ang   	#taking care of negative angles
	if ang>2*3.1415:
		ang=ang-2*3.1415		#taking care of angles > 360 degrees

	
	x=x*100-(tr*100)*math.cos(ang)
	y=y*100-(tr*100)*math.sin(ang)
	x,y=cell(x,y)
	z=c
	l=[x,y,z]
	temp=1.0
	grid=gaussian2(temp,l)
	m.grid=m.grid+grid
	for i in range(0,35):
		for j in range (0,35):
			for k in range (0,36):
				m.grid[i,j,k]=m.grid[i,j,k]/2




def visual(pub2,p,q,i,marker):
	p=float(p)
	q=float(q)
	x=float(p/100)
	y=float(q/100)
	for n in range(0,2):
		
		marker.header.frame_id = "/base_link"
		marker.header.stamp = rospy.Time.now()
		marker.ns="linestrip"
		marker.type=marker.LINE_STRIP
		marker.action = marker.ADD
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 1.0
		marker.scale.z = 1.0
		marker.color.r = 1.0
		marker.color.a = 1.0
		marker.lifetime=rospy.Duration()
		marker.id=i
		threePoints=[]	
		p=Point()
		p.x=x
		p.y=y
		
		marker.points.append(p)
		pub2.publish(marker)


def coord(x,y):
	x2=(x+1)*20-10	
	y2=(y+1)*20-10
	return x2,y2

def cell(x,y):			# x,y in cm
	x=round(x/20)
	if x>34:
		x=34
	elif x<0:
		x=0
	y=round(y/20)
	if y>34:
		y=34
	elif y<0:
		y=0	
	return x,y

def printme():
	maxval=np.amax(m.grid)			#max of grid
	count=0
	for i in range(0,35):
		for j in range (0,35):
			for k in range (0,36):
				if m.grid[i,j,k]==maxval and count==0	:
					p,q,r=i,j,k
					count=count+1
					
	return p,q,r

def getmax():
	maxval=np.amax(m.grid)			#max of grid
	count=0
	for i in range(0,35):
		for j in range (0,35):
			for k in range (0,36):
				if m.grid[i,j,k]==maxval and count==0	:
					p,q,r=i,j,k
					count=count+1

	return p,q				


if __name__ =="__main__":
	

	try:
		rospy.init_node('gridloc',anonymous=True)
		rate = rospy.Rate(10)



		#Data structure to store probablities
		class matrix(object):
			def __init__(self):
				self.name='class Matrix'
				self.grid=np.zeros((35,35,36))
				self.tags=[[1.25,5.25],[1.25,3.25],[1.25,1.25],[4.25,1.25],[4.25,3.25],[4.25,5.25]]
				self.recent=[]


		#Initializations
		m=matrix()
		#Initial pose is (11,27,20)
		m.grid[11,27,20]=1.0



     

		
		pub1 = rospy.Publisher('/visualization_marker',Marker, queue_size = 100)
		count=0
		while count<10000:
			co=m.tags[count%6]
			x=co[0]
			y=co[1]     
			marker = Marker()
			marker.header.frame_id = "/base_link"
			marker.header.stamp = rospy.Time.now()
			marker.ns="cubes"
			marker.type=marker.POINTS
			marker.action = marker.ADD
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = 0.2
			marker.scale.y = 0.2
			marker.scale.z = 0.2
			marker.scale.z = 1.0
			marker.color.g = 1.0
			marker.color.a = 1.0
			marker.lifetime=rospy.Duration()
			marker.id=count%6
			threePoints=[]	
			p=Point()
			p.x=x
			p.y=y
			p.z=0
			threePoints.append(p)
			marker.points=threePoints
			pub1.publish(marker)
			count=count+1




		pub2 = rospy.Publisher('/visualization_marker',Marker, queue_size = 100)	
		marker = Marker()
		#Start of the program
		for i in range (0,89):					#Should be (0,89)
			movop(mov[i])
			#print "M:",printme()
			obsop(obs[i])
			#print "O:",printme()
			p,q=getmax()
			p,q=coord(p,q)
			visual(pub2,p,q,i,marker)




		print np.amax(m.grid)
		print np.sum(m.grid)
		print "Done"

		


	except rospy.ROSInterruptException:
        	pass
	
	rospy.spin()	