#!/usr/bin/env python
# coding: utf-8

import cvxopt
from cvxopt import matrix, printing
import numpy
from vedo import *

import time



class RobotArm():
	'''
		Robot arm.
	'''
	
	def __init__(self, partLengths, parts, arm_location):
		'''
			Initialize object
		''' 

		# Arm location (position of the first frame)
		self.arm_location = arm_location

		# Length of the parts 
		self.L1, self.L2, self.L3, self.L4 = partLengths
		
		# Constants
		self.delta_phi = 0.1
		self.target = [0, 100, 200]
		self.target_tolerance = 30
		self.target_lambda = 0.001
		self.convergence = 0.02
		self.iteration_limit = 3000

	"""
	def buildAllParts(self):
		'''
			Build the robot mesh with all parts. 
			Robot is created in its neutral pose. 
		'''

		# Construct the arm parts 
		self.Part1 = self.createArmPartMesh(self.L1)	
		self.Part2 = self.createArmPartMesh(self.L2)			
		self.Part3 = self.createArmPartMesh(self.L3)			
		self.Part4 = self.createCoordinateFrameMesh()  # End effector (not an actual frame part)			
	"""

	def setPose(self, Phi, parts):
		'''
			Set pose of the robot arm.  
		'''

		# Obtain local-to-global matrices from forward kinematics
		T_00, T_01, T_02, T_03, T_04, e = self.forward_kinematics(Phi)

		# Re-create robot in its neural position
		# self.buildAllParts()

		# Transforms parts to position it at its correct
		# location and orientation. 
		Base = parts[0].clone()
		Part1 = parts[1].clone()
		Part2 = parts[2].clone()
		Part3 = parts[3].clone()
		Part4 = self.createCoordinateFrameMesh()  # End effector (not an actual frame part)

		Base.apply_transform(T_00)
		Part1.apply_transform(T_01)  
		Part2.apply_transform(T_02)  
		Part3.apply_transform(T_03)  
		Part4.apply_transform(T_04)  

		return [Base, Part1, Part2, Part3, Part4]


	def RotationMatrix(self, theta, axis_name):
		""" calculate single rotation of $theta$ matrix around x,y or z
		# code from: https://programming-surgeon.com/en/euler-angle-python-en/
		input
	        theta = rotation angle(degrees)
	        axis_name = 'x', 'y' or 'z'
	    output
	        3x3 rotation matrix
	    """

		c = np.cos(theta * np.pi / 180)
		s = np.sin(theta * np.pi / 180)
		
		if axis_name =='x':
			rotation_matrix = np.array([[1, 0,  0],
	                                    [0, c, -s],
	                                    [0, s,  c]])
		if axis_name =='y':
			rotation_matrix = np.array([[ c,  0, s],
	                                    [ 0,  1, 0],
	                                    [-s,  0, c]])
		elif axis_name =='z':
			rotation_matrix = np.array([[c, -s, 0],
	                                    [s,  c, 0],
	                                    [0,  0, 1]])
		return rotation_matrix

	def createCoordinateFrameMesh(self):
		"""Returns the mesh representing a coordinate frame
	    Args:
	      No input args
	    Returns:
	      F: vedo.mesh object (arrows for axis)
		"""
		_shaft_radius = 0.05
		_head_radius = 0.10
		_alpha = 1
		unit = 30
	    
	    # x-axis as an arrow  
		x_axisArrow = Arrow(start_pt=(0, 0, 0),
	                        end_pt=(unit, 0, 0),
	                        s=None,
	                        shaft_radius=_shaft_radius,
	                        head_radius=_head_radius,
	                        head_length=None,
	                        res=12,
	                        c='red',
	                        alpha=_alpha)

	    # y-axis as an arrow  
		y_axisArrow = Arrow(start_pt=(0, 0, 0),
	                        end_pt=(0, unit, 0),
	                        s=None,
	                        shaft_radius=_shaft_radius,
	                        head_radius=_head_radius,
	                        head_length=None,
	                        res=12,
	                        c='green',
	                        alpha=_alpha)

	    # z-axis as an arrow  
		z_axisArrow = Arrow(start_pt=(0, 0, 0),
	                        end_pt=(0, 0, unit),
	                        s=None,
	                        shaft_radius=_shaft_radius,
	                        head_radius=_head_radius,
	                        head_length=None,
	                        res=12,
	                        c='blue',
	                        alpha=_alpha)
	    
		originDot = Sphere(pos=[0,0,0], 
	                       c="black", 
	                       r=0.10*unit)


	    # Combine the axes together to form a frame as a single mesh object 
		F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
	        
		return F


	def getLocalFrameMatrix(self, R_ij, t_ij): 
		"""Returns the matrix representing the local frame
	    Args:
	      R_ij: rotation of Frame j w.r.t. Frame i 
	      t_ij: translation of Frame j w.r.t. Frame i 
	    Returns:
	      T_ij: Matrix of Frame j w.r.t. Frame i. 
	      
	    """             
		# Rigid-body transformation [ R t ]
		T_ij = np.block([[R_ij,                t_ij],
	                     [np.zeros((1, 3)),       1]])
	    
		return T_ij

	
	# def createArmPartMesh(self, L):

	# 	# Create a sphere to show as an example of a joint
	# 	radius = 0.4
	# 	sphere1 = Sphere(r=radius).pos(0,0,0).color("gray").alpha(.8)
		
	# 	# Create the coordinate frame mesh and transform
	# 	Frame1Arrows = self.createCoordinateFrameMesh()
		
	# 	# Now, let's create a cylinder and add it to the local coordinate frame
	# 	link1_mesh = Cylinder(r=0.4, 
	# 	                      height=L, 
	# 	                      pos = (0,0,L/2+radius),
	# 	                      c="white", 
	# 	                      alpha=.8, 
	# 	                      axis=(0,0,1)
	# 	                      )
		
	# 	# Combine all parts into a single object 
	# 	Part = Frame1Arrows + link1_mesh + sphere1
		
	# 	return Part
	

	def forward_kinematics(self, Phi):
		
		# Radius of the sphere representing the joint
		radius = 0.4
		
		# Joint angle 
		phi1 =  Phi[0]    # Rotation angle of part 1 in degrees
	

		# Matrix of Robot Base (written w.r.t. Coordinate origin, which is the previous frame) 
		R_00 = self.RotationMatrix(0, axis_name = 'z')   	# Rotation matrix, 0 rotation
		t_00 = np.copy(self.arm_location)                            # Frame's origin (w.r.t. previous frame)

		t_00[-1] = 0
		T_00 = self.getLocalFrameMatrix(R_00, t_00)         # Matrix of Robot Base w.r.t. Coordinate origin 


		# Matrix of Frame 1 (written w.r.t. Frame 0, which is the previous frame) 
		R_01 = self.RotationMatrix(phi1, axis_name = 'z')   # Rotation matrix
		t_01 = self.arm_location                            # Frame's origin (w.r.t. previous frame)
		T_01 = self.getLocalFrameMatrix(R_01, t_01)         # Matrix of Frame 1 w.r.t. Frame 0 

		# Joint angle 
		phi2 = Phi[1]    # Rotation angle of part 2 in degrees

		# Matrix of Frame 2 (written w.r.t. Frame 1, which is the previous frame) 	
		R_12 = self.RotationMatrix(phi2, axis_name = 'y')   # Rotation matrix
		t_12 = np.array([[0.0], [0.0], [self.L1+2*radius]])  # Frame's origin (w.r.t. previous frame)
		T_12 = self.getLocalFrameMatrix(R_12, t_12)         # Matrix of Frame 2 w.r.t. Frame 1 
		
		# Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
		T_02 = T_01 @ T_12


		phi3 = Phi[2]    # Rotation angle of the end-effector in degrees
			
		# Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame) 	
		R_23 = self.RotationMatrix(phi3, axis_name = 'y')   # Rotation matrix
		t_23   = np.array([[0.0], [0.0], [self.L2+2*radius]])  # Frame's origin (w.r.t. previous frame)
		
		# Matrix of Frame 3 w.r.t. Frame 2 
		T_23 = self.getLocalFrameMatrix(R_23, t_23)
		
		# Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
		T_03 = T_01 @ T_12 @ T_23

		phi4 = Phi[3]
			
		# Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame) 	
		R_34 = self.RotationMatrix(phi4, axis_name = 'y')   # Rotation matrix
		t_34   = np.array([[-28.4], [0.0], [self.L3+radius]])  # Frame's origin (w.r.t. previous frame)
		
		# Matrix of Frame 3 w.r.t. Frame 2 
		T_34 = self.getLocalFrameMatrix(R_34, t_34)
		
		# Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
		T_04 = T_01 @ T_12 @ T_23 @ T_34

		e = T_04[0:3,-1]    # Last column of the last frame matrix

		return T_00, T_01, T_02, T_03, T_04, e

	def jacobian_matrix(self, phi):

		step = self.delta_phi * np.pi / 180 # radians
		# Obtain local-to-global matrices from forward kinematics
		_, _, _, _, _, e = self.forward_kinematics(phi)
		# phi 1 gradient 
		_, _, _, _, _, e_phi1_delta = self.forward_kinematics(phi + np.array([step, 0, 0, 0]))
		e_phi1_derive = (e_phi1_delta - e) / step
		# phi 2 gradient
		_, _, _, _, _, e_phi2_delta = self.forward_kinematics(phi + np.array([0, step, 0, 0]))
		e_phi2_derive = (e_phi2_delta - e) / step
		# phi 3 gradient
		_, _, _, _, _, e_phi2_delta = self.forward_kinematics(phi + np.array([0, 0, step, 0]))
		e_phi3_derive = (e_phi2_delta - e) / step

		jacobian = np.concatenate((e_phi1_derive.reshape((3,1)), e_phi2_derive.reshape((3,1)), e_phi3_derive.reshape((3,1))), axis=1)
		# print(e_phi1_derive)
		# print(jacobian)
		return jacobian

	def snapshot(self, recorder, phi, parts):
		Part = self.setPose(phi, parts)
		recorder.append(Part)

	def inverse_kinematics_gradient_descent(self, initial_phi, parts):
		phi  = initial_phi
		_, _, _, _, _, e = self.forward_kinematics(initial_phi)
		target = self.target
		target_lambda = self.target_lambda
		convergence = self.convergence
		target_tolerance = self.target_tolerance
		iteration_limit = self.iteration_limit
		recorder = []
		iteration = 0
		e_accumulate = 0
		
		while abs(numpy.linalg.norm(target - e)) > target_tolerance:
			iteration += 1 
			jacobian_matrix = self.jacobian_matrix(phi)
			jacobian_pseudo_inverse = np.linalg.pinv(jacobian_matrix)
			e_delta = target_lambda*(target - e)
			phi_delta = jacobian_pseudo_inverse @ e_delta
			phi_delta = np.append(phi_delta, [0], axis = 0)
			phi = phi + phi_delta
			e_previous = e
			_, _, _, _, _, e = self.forward_kinematics(phi)
			e_accumulate += numpy.linalg.norm(e_previous - e)

			# print(abs(numpy.linalg.norm(target - e)))
			# print(abs(numpy.linalg.norm(e_previous - e)))

			# snap shot
			if iteration % 20 == 0 or e_accumulate > 10:
				self.snapshot(recorder, phi, parts)
				e_accumulate = 0

			# gradient descent exit criteria 1
			if abs(numpy.linalg.norm(e_previous - e)) < convergence:
				break

			# gradient descent exit criteria 2
			if iteration > iteration_limit:
				break
		
		return recorder


			

def main():
	"""
	Settings
	"""	
	# component heights in mm, need to get aligned with stl models
	unit = 1
	BaseH = 105/unit
	BaseRotH = 81/unit
	HumerusH = 217/unit
	RadiusH = 416/unit

	BaseX = 10/unit
	BaseY = 10/unit

	Circle_radius = 550/unit

	BasePos = [ [Circle_radius, 0],
			 	[Circle_radius * np.cos(120 * np.pi / 180), Circle_radius * np.sin(120 * np.pi / 180)],
				[Circle_radius * np.cos(-120 * np.pi / 180), Circle_radius * np.sin(-120 * np.pi / 180)]
				]

	L = [BaseRotH, HumerusH, RadiusH, 0]
	# Animation parameters
	view_angle = [1,1,1] 
	degree1Max = 60
	degree2Max = 60
	degree3Max = 60

	step = 5

	# load parts from stl file
	Base = load('./Base.stl').color('blue5')
	BaseRot = load('./BaseRot.stl').color('lightblue')
	Humerus = load('./Humerus.stl').color('gray5')
	Radius = load('./Radius.stl').color('red5')
	
	parts = [Base, BaseRot, Humerus, Radius]

	# arm_location = np.array([[BaseX],[BaseY], [BaseH]])    	# Arm location (position of the first frame)
	arm_location0 = np.array([[BasePos[0][0]],[BasePos[0][1]], [BaseH]])    	# Arm location (position of the first frame)
	arm_location1 = np.array([[BasePos[1][0]],[BasePos[1][1]], [BaseH]])    	# Arm location (position of the first frame)
	arm_location2 = np.array([[BasePos[2][0]],[BasePos[2][1]], [BaseH]])    	# Arm location (position of the first frame)

	# create instances
	myRobot0 = RobotArm(L, parts, arm_location0)             # The robot arm 
	myRobot1 = RobotArm(L, parts, arm_location1)             # The robot arm 
	myRobot2 = RobotArm(L, parts, arm_location2)             # The robot arm 

	# Set the limits of the graph x, y, and z ranges
	axes = Axes(xrange=(-Circle_radius*1.2,Circle_radius*1.2), yrange=(-Circle_radius*1.2,Circle_radius*1.2), zrange=(0,Circle_radius*1.2))
	circle = Circle(pos=(0, 0, 0), r=Circle_radius, res=120, c='lightgray', alpha=1.0)
	# declare the class instance
	plt = Plotter(bg='beige', bg2='lb', axes=10, offscreen=False, interactive=False)
	plt.show(axes, viewup = view_angle)

	# A short sequence of poses 
	Poses = np.array([[  0,  0,  0,  0],
					  [-30, 50, 30,  0],
					  [ 30,-50,-30,  0],
					  [  0,  0,  0,  0]])
	snapshot0 = myRobot0.inverse_kinematics_gradient_descent(Poses[0], parts)
	snapshot1 = myRobot1.inverse_kinematics_gradient_descent(Poses[0], parts)
	snapshot2 = myRobot2.inverse_kinematics_gradient_descent(Poses[0], parts)

	ball = Sphere(myRobot0.target, r=30).c("red")

	idxlimit = max(len(snapshot0), len(snapshot1), len(snapshot2))
	
	# animation
	for idx in range(idxlimit):
		# All objects to Plotter.
		# Clear plot before showing new iteration
		plt.clear()
		plt += axes
		plt += circle
		plt += ball
		if idx >= len(snapshot0):
			plt += snapshot0[len(snapshot0)-1]
		else: 
			plt += snapshot0[idx]
		if idx >= len(snapshot1):			
			plt += snapshot1[len(snapshot1)-1]
		else:
			plt += snapshot1[idx]
		if idx >= len(snapshot2):			
			plt += snapshot2[len(snapshot2)-1]
		else:
			plt += snapshot2[idx]

		idx += 1
		# Show scene    
		plt.render()    #  What is the difference between render() and show()? 
		screenshot('snapshot_%03d.png' %idx)


if __name__ == '__main__':
    main()



