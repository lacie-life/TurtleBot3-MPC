 #!/usr/bin/env python

import rospy
import tf
import csv
from csv import writer
import numpy as np
from numpy import genfromtxt, savetxt
import matplotlib as plt
from turtlebot_mpc.unicycle_mpc import UnicycleMPC
from geometry_msgs.msg import Twist, Pose, Quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool

class TurtleBotMPC(object):
   def __init__(self, cfg):
      self.cfg = cfg
      
      self.T = cfg['mpc']['T']
      self.N = int(cfg['mpc']['N'])
      
      if cfg['perpendicular']:
         self.xmin = np.array(cfg['mpc']['xmin1']).astype(np.float32)
         self.xmax = np.array(cfg['mpc']['xmax1']).astype(np.float32)
         self.umin = np.array(cfg['mpc']['umin1']).astype(np.float32)
         self.umax = np.array(cfg['mpc']['umax1']).astype(np.float32)

         self.Q = np.array(cfg['mpc']['Q1']).astype(np.float32)
         self.QN = np.array(cfg['mpc']['QN1']).astype(np.float32)
         self.R = np.array(cfg['mpc']['R1']).astype(np.float32)

      if cfg['parallel']:
         self.xmin = np.array(cfg['mpc']['xmin2']).astype(np.float32)
         self.xmax = np.array(cfg['mpc']['xmax2']).astype(np.float32)
         self.umin = np.array(cfg['mpc']['umin2']).astype(np.float32)
         self.umax = np.array(cfg['mpc']['umax2']).astype(np.float32)

         self.Q = np.array(cfg['mpc']['Q2']).astype(np.float32)
         self.QN = np.array(cfg['mpc']['QN2']).astype(np.float32)
         self.R = np.array(cfg['mpc']['R2']).astype(np.float32)

      self.mpc = UnicycleMPC(self.T, self.N, self.xmin, self.xmax, self.umin, self.umax, self.Q, self.QN, self.R)

      self.init = False
      self.model_name = cfg['model_name']
      self.model_found = False
      self.model_index = 0

      self.xstate = np.zeros(3)

      self.model_states_sub = rospy.Subscriber(self.cfg['model_states_topic'], ModelStates, self.model_state_cb, queue_size=1)
      self.wait_sub = rospy.Subscriber(self.cfg['wait_topic'], Bool, self.wait_for_signal_cb, queue_size=1)
      self.cmd_pub = rospy.Publisher(cfg['command_topic'], Twist, queue_size=1)

      if cfg['debug']:
         self.make_test_trajectory()
      else:
         self.reference_trajectory(cfg)

      rospy.loginfo("TurltebotMPC: Initialized")

   def model_state_cb(self, msg):
      """Retrieve model states for turtlebot from gazebo"""
      if not self.model_found:
         for i in range(len(msg.name)):
            if msg.name[i] == self.model_name:
               self.model_index = i
               self.model_found = True

      if not self.model_found:
         rospy.logwarn("TurtleBotMPC: Model '%s' not found in '%s'", self.model_name, self.cfg['model_states_topic'])
         return

      # Grab current vehicle state
      pose = msg.pose[self.model_index]
      quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
      rpy = tf.transformations.euler_from_quaternion(quat)
      self.xstate[0] = pose.position.x
      self.xstate[1] = pose.position.y
      self.xstate[2] = rpy[2]

   def wait_for_signal_cb(self, msg):
      """Ensures robot doesn't start moving until told to do so"""
      if msg.data:
         self.init = True
         rospy.loginfo("Turlebot MPC: Go signal received")

   def make_test_trajectory(self):
      xf = 2.0
      xvel = 0.2
      Tf = xf / xvel
      Kf = Tf / self.T

      zero3 = np.zeros((1,3))
      zero2 = np.zeros((1,2))

      xref = zero3
      uref = zero2
      for k in range(1, int(Kf)+1):
         xk1 = np.array([[xvel * self.T * float(k), 0., 0.]])
         uk1 = np.array([[xvel, 0.]])
         xref = np.concatenate([xref, xk1])
         uref = np.concatenate([uref, uk1])
      uref = np.delete(uref, 0, 0)
      print(len(xref))
      print(len(uref))
      self.mpc.set_ref_trajectory(xref, uref)
      print("TurtleBotMPC: Test ref trajectory set")

   def reference_trajectory(self,cfg):
      """
      xref(0)  --> xref(Kf) 
      uref(0)  --> uref(Kf-1) 
      """

      #with open(xref_file_path, 'r') as f:
      #   reader = csv.reader(f, delimiter=',')
      #   headers = next(reader)
      #   xref = np.array(list(reader)).astype(float)
      #with open(uref_file_path, 'r') as f:
      #   reader = csv.reader(f, delimiter=',')
      #   headers = next(reader)
      #   uref = np.array(list(reader)).astype(float)
      #self.mpc.set_ref_trajectory(xref, uref)

      if cfg['perpendicular']:
         xref_file_path = rospy.get_param("~xref_perp_path")
         uref_file_path = rospy.get_param("~uref_perp_path")
         xref = genfromtxt(xref_file_path, delimiter=',')
         uref = genfromtxt(uref_file_path, delimiter=',')

      if cfg['parallel']:
         xref_file_path = rospy.get_param("~xref_par_path")
         uref_file_path = rospy.get_param("~uref_par_path")
         xref = genfromtxt(xref_file_path, delimiter=',')
         uref = genfromtxt(uref_file_path, delimiter=',')

      self.mpc.set_ref_trajectory(xref, uref)
      print("TurtleBotMPC: Ref trajectory set")
   
   def run_controller(self):
      """Run MPC controller"""
      if not self.model_found or not self.init:     
         return 
      
      #rospy.loginfo("Turtlebot MPC: Running MPC")
      status, u = self.mpc.update(self.xstate)
      u = u.flatten()
      #print("Control")
      print(u)
      #xk = (self.xstate).reshape(1,-1)
      #uk = u.reshape(1,-1)
      #print(xk)
      #np.savetxt('mpc_state.csv',xk,delimiter=',')
      #np.savetxt('mpc_input.csv',uk,delimiter=',')
      cmd = Twist()
      cmd.linear.x = u[0]
      cmd.angular.z = u[1]
      self.cmd_pub.publish(cmd)
      #self.append_list_as_row('mpc_state.csv', xk)

   #def append_list_as_row(self,file_name, list_of_elem):
   #   self.file_name = file_name
   #   self.list_of_elem = list_of_elem
      
      #Method 1
      #with open(file_name, "a") as myfile:
      #   myfile.write(list_of_elem)
      
      #Method 2
      # Open file in append mode
   #   with open(file_name, 'a') as write_obj:
         # Create a writer object from csv module
   #      csv_writer = writer(write_obj)
         # Add contents of list as last row in the csv file
   #      csv_writer.writerow(list_of_elem)
      
      
      
