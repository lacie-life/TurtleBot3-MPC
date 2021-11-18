#!usr/bin/env python

import math
import numpy as np
# import cvxopt
import cvxpy as cp

class UnicycleMPC(object):

   def __init__(self, T, N, xmin, xmax, umin, umax, Q, QN, R):
      """Constructor

      Notation:
         Kf = final kth time step in the reference trajectory
         n = 3 = number of states (x [m], y [m], theta [rad])
         m = 2 = number of control inputs (fwd_vel [m/s], ang_vel [rad/s])

      Assumptions
         1. Robot follows a kinematic unicycle model when using small time-steps
         2. Reference trajectory (xref) ends at rest. That is, xref(k=Kf) = 0

      Args:
         T: Time step (double)
         N: Receding horizon (integer)
         xmin: Min values of (x - xref) as numpy array of size (1,3)
         xmax: Max values of (x - xref) as numpy array of size (1,3)
         umin: Min values of control input u as numpy array of size (1,2)
         umax: Max values of control input u as numpy array of size (1,2)
         Q: State cost matrix as numpy array of size (n,n)
         QN: Final state cost matrix as numpy array of size (n,n)
         R: Input cost matrix as numpy array of size (m,m)
      """
      self.n = 3 # 3 states (x, y, theta)
      self.m = 2 # 2 inputs (fwd_vel, ang_vel)
      self.k = 0 # Time step k

      self.T = T
      self.N = N
      self.xmin = xmin.reshape(1,3)
      self.xmax = xmax.reshape(1,3)
      self.umin = umin.reshape(1,2)
      self.umax = umax.reshape(1,2)
      self.Q = Q
      self.QN = QN
      self.R = R

      self.set_QR_cost_matrices()
      self.init = False
      
   def set_QR_cost_matrices(self):
      """Setup Qbar and Rbar matrices needed by QP problem"""

      self.Qbar = np.zeros((self.n * self.N, self.n * self.N))
      self.Rbar = np.zeros((self.m * self.N, self.m * self.N))
      
      for i in range(self.N-1):
         j = i * self.n
         k = i * self.m
         self.Qbar[j:j+self.n, j:j+self.n] = self.Q   #math.pow(2,i)*self.Q
         self.Rbar[k:k+self.m, k:k+self.m] = self.R

         if i == self.N-2:
            j = j + self.n
            k = k + self.m
            self.Qbar[j:j+self.n, j:j+self.n] = self.QN    #math.pow(2,i+1)*self.Q
            self.Rbar[k:k+self.m, k:k+self.m] = self.R

   def set_ref_trajectory(self, xref, uref):
      """Set the reference trajectory

      Notation:
         Kf = final kth time step in the reference trajectory

      Notes:
         1. Do NOT pad the reference trajectory at the end with zeros (this code will take care of that)
         2. Reference trajectory (xref) must at rest. That is, xref(k=Kf) = 0

      Args:
         xref: Entire reference trajectory from x0 to xf as numpy array of size (Kf+1, 3), where each row is (xref [m], yref [m], theta_ref[rad])
               Note: row 0 corresponds to xref(k=0)
                     row Kf corresponds to xref(k=Kf)
                     padded rows will be equal to xref(k=Kf)
         uref: Entire reference input from ur0 to urf as numpy array of size (Kf, 2), where each row is (vref [m/s], ang_vel_ref [rad/s])
               Note: row 0 corresponds to uref(k=0)
                     row Kf-1 corresponds to last control input
                     padded rows will be equal to all 0s
      """
      if xref.shape[0] != uref.shape[0] + 1:
         print("Xref shape should have one more row than uref")
         return False
      
      self.xref = xref
      self.uref = uref
      self.Kf = self.uref.shape[0]
      
      # Pad ref trajectory at end with zero rows
      for i in range(self.N):
         self.xref = np.append(self.xref, np.zeros((1,3)), axis=0)
         self.uref = np.append(self.uref, np.zeros((1,2)), axis=0)
      
      self.init = True
      self.k = 0

      return True

   def Ak(self, k):
      """Discrete LTV system: x(k+1) = A(k)*x(k) + B(k)*u(k)
         Return A(k)

      Args:
         k: kth instant

      Returns:
         Matrix A(k) of size (n,n)
      """
      A = np.identity(3)
      A[0,2] = -self.uref[k,0] * math.sin(self.xref[k,2]) * self.T
      A[1,2] = self.uref[k,0] * math.cos(self.xref[k,2]) * self.T
      return A

   def Bk(self, k):
      """Discrete LTV system: x(k+1) = A(k)*x(k) + B(k)*u(k)
         Return B(k)

      Args:
         k: kth instant

      Returns:
         Matrix B(k) of size (n,m)
      """
      B = np.zeros((3,2))
      B[0,0] = math.cos(self.xref[k,2]) * self.T
      B[1,0] = math.sin(self.xref[k,2]) * self.T
      B[2,1] = self.T
      return B

   def Akjl(self, k, j, l):
      """Returns A(k,j,l) = A(k+N-j) * A(k+N-j-1) * A(k+N-j-2) * ... * A(k+l)

      Args:
         k: kth instant
         j: Defines the first element in the multiplication at (k+N-j)
         l: Defines the last element in the multiplication at (k+l)

      Returns:
         Matrix A(k,j,l) of size (n,n)
      """
      Akjl = np.identity(3)
      for i in reversed(range(self.N - j - l + 1)):
         Akjl = Akjl.dot(self.Ak(k + i))
      return Akjl

   def Abar(self, k):
      """Get Abar matrix at kth instant

      Args:
         k: kth instant

      Returns:
         Matrix Abar(k) of size (nN, n)
      """
      Abar = self.Akjl(k, self.N, 0) # A(k)
      for j in reversed(range(1, self.N)): # N-1, N-2, ..., 1
         Abar = np.vstack((Abar, self.Akjl(k,j,0)))
      return Abar

   def Bbar(self, k):
      """Get Bbar matrix at kth instant

      Args:
         k: kth instant

      Returns:
         Matrix Bbar(k) of size (nN, mN)
      """
      Bbar = np.zeros((self.n * self.N, self.m * self.N))

      # Fill in the block diagonal elements
      for q in range(self.N):
         i = q * self.n
         j = q * self.m
         Bbar[i:i+self.n, j:j+self.m] = self.Bk(k+q)

      # Now fill in the lower triangular portion
      for r in range(self.N-1): # Across the col groups 0, 1, ,,, N-2
         j = r * self.m
         
         Bkr = self.Bk(k+r)
         for q in range(r+1, self.N): # Down the row groups 1, 2, ..., N-1
            i = q * self.n
            Bbar[i:i+self.n, j:j+self.m] = self.Akjl(k, self.N-q, r+1).dot(Bkr)
      return Bbar

   def update(self, xk):
      """Update model with current xstate, compute control input u(k)

      Solve MPC problem as a QP problem for trajectory tracking
      QP Problem: min 0.5 * u^T * H * u + f^T * u^T
                  subject to: G * u <= w

                  where u = u(k) - uref(k)
      
      Args:
         xk: current state x(k) as numpy array of size (1,3), do NOT subtract xref(k)

      Returns (multiple arguments):
         Boolean indicating problem was solved
         Control input u(k) = uerr + uref(k) to apply to the system as numpy (2,1) array (Note: uref has been added back)
      """
      if not self.init:
         return False, np.zeros((2,1))
      
      if self.k >= self.Kf:
         return True, np.zeros((2,1))

      xerr = (xk.reshape(1,self.n) - self.xref[self.k,:]).reshape(self.n,1)
      Abar = self.Abar(self.k)
      Bbar = self.Bbar(self.k)

      # QP matrices
      # Recall, np.dot is normal matrix multiplication
      H = 2 * Bbar.T.dot(self.Qbar).dot(Bbar) + self.Rbar # Size (mN, mN)
      f = 2 * Bbar.T.dot(self.Qbar).dot(Abar).dot(xerr) # Size (mN, 1)

      # G matrix
      ImN = np.identity(self.m * self.N)
      G = np.concatenate((ImN, -ImN, Bbar, -Bbar)) # Size (2mN + 2nN, mN)

      # W Vector
      w1 = (self.umax - self.uref[self.k:self.k + self.N, :]).flatten().reshape(-1,1) # Flatten into 2D col vector
      w2 = (-self.umin + self.uref[self.k:self.k + self.N, :]).flatten().reshape(-1,1)
      # w3 = (self.xmax - self.xref[self.k:self.k + self.N, :]).flatten().reshape(-1,1) - Abar.dot(xerr)
      # w4 = (-self.xmin + self.xref[self.k:self.k + self.N, :]).flatten().reshape(-1,1) + Abar.dot(xerr)
      w3 = np.tile(self.xmax.reshape(-1,1), (self.N, 1)) - Abar.dot(xerr)
      w4 = np.tile(-self.xmin.reshape(-1,1), (self.N, 1)) + Abar.dot(xerr)
      w = np.concatenate([w1, w2, w3, w4]) # Size (2mN + 2nN, 1)

      # # Setup and solve QP problem
      # H = cvxopt.matrix(Hk)
      # f = cvxopt.matrix(fk)
      # G = cvxopt.matrix(Gk)
      # w = cvxopt.matrix(wk)

      # cvxopt.solvers.options['show_progress'] = False
      # u = cvxopt.solvers.qp(H, f, G, w)
      # self.k = self.k + 1
      # return u[0:self.m] + self.uref[self.k-1, :].reshape(2,1)

      # Solve with cvxpy
      u = cp.Variable((self.m * self.N, 1))
      prob = cp.Problem(cp.Minimize(0.5*cp.quad_form(u, H) + f.T * u), [G * u <= w])
      prob.solve()
      self.k = self.k + 1

      if prob.status not in ["infeasible", "unbounded"]:
         return True, u[0:self.m, 0].value.reshape(-1,1) + self.uref[self.k-1, :].reshape(-1,1)
      else:
         print("MPC solver: %s" %(prob.status))
         return False, np.zeros((2,1))
      
