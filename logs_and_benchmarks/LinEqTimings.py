import numba
import timeit
import numpy as np
from scipy import linalg as sla
from scipy.linalg import lu_factor, lu_solve

from RI import getRayIntersection

def gen_ex(num_segs):
  n = num_segs*2
  #x = np.random.randn(d0,d0)
  A = gen_sparse_A(n)
  t = np.random.randn(n,1)
  b = np.random.randn(n)
  t = np.random.randn(n, 1)
  x1 = A[:2,:1]
  x2 = np.random.randn(1, 2)
  x3 = np.random.randn(2, 1)
  return A, x1, x2, x3, t, b

def gen_sparse_A(n):
  # init A matrix
  A = np.zeros((n, n))

  # fill with random values along diagonal
  for i in range(int(np.floor((n/2)))):
    idx = i*2
    A[idx,idx] = np.random.randn()
    A[idx,idx+1] = np.random.randn()
    A[idx+1,idx] = np.random.randn()
    A[idx+1,idx+1] = np.random.randn()
  return A

def gen_ex_RayIntersection(num_segs):
  COSALPHA = 1
  SINALPHA = 1
  lidar_x = 1
  lidar_y = 1
  _d_max = 1000
  line_vecs = np.array(np.random.randn(num_segs, 2))
  vertices = np.array(np.random.randn(num_segs, 2))
  return COSALPHA, SINALPHA, lidar_x, lidar_y, _d_max, num_segs, vertices, line_vecs

def matmul_nla(A, t):
  return np.linalg.inv(A) @ t

def matmul_sla(A, t):
  return sla.inv(A) @ t

@numba.jit
def matmul_nla_jit(A, t):
  return np.linalg.inv(A) @ t

def matmul_lu(A, t):
  lu, piv = lu_factor(A)
  x = lu_solve((lu, piv), t)
  return x

def matmul_small_analytical(x1, x2, x3, num_segments):
  for _ in range(num_segments):
    det = np.dot(x1, x2)  # det is the determinant of the A matrix
    res1 = np.dot(x3[::-1].T, x1) / det  # d
    res2 = np.dot(x2, x1) / det  # lambda, [::-1] flips array
  return res1, res2

@numba.jit
def matmul_small_analytical_jit(x1, x2, x3, num_segments):
  for _ in range(num_segments):
    det = np.dot(x1, x2)  # det is the determinant of the A matrix
    res1 = np.dot(x3[::-1].T, x1) / det  # d
    res2 = np.dot(x2, x1) / det  # lambda, [::-1] flips array
  return res1, res2

@numba.jit
def matmul_small_nla_jit(A, t, num_segments):
  for _ in range(num_segments):
    det = np.linalg.det(A) #needed to sort out parallel lines which would lead to a singular matrix error
    res = np.linalg.inv(A) @ t
  return res

@numba.jit
def solve_small_nla_jit(A, t, num_segments):
  for _ in range(num_segments):
    det = np.linalg.det(A)  #needed to sort out parallel lines which would lead to a singular matrix error
    res2 = np.linalg.solve(A, t)
  return res2

#Code for IPyhthon console

#------big matrix methods-------

# A, x1, x2, x3, t, b = gen_ex(num_segments) # n is number of dimentions of A matrix = #segments*2

# %timeit matmul_nla(A,t)

# %timeit matmul_sla(A,t)

# %timeit matmul_nla_jit(A,t)

# %timeit matmul_lu(A,t)

#------small matrix methods-------

# A, x1, x2, x3, t, b = gen_ex(2) # #2x2 A matix is needed because 2x2 matrix is solved num_segments times

# %timeit matmul_small_analytical(x1, x2, x3, num_segments)

# %timeit matmul_small_analytical_jit(x1, x2, x3, num_segments)

# %timeit matmul_small_nla_jit(A, t, num_segments)

# %timeit solve_small_nla_jit(A, t, num_segments)
