from sympy import *
import numpy as np


x1 = Symbol('x1')
x2 = Symbol('x2')
x3 = Symbol('x3')
x4 = Symbol('x4')
y1 = Symbol('y1')


x_array = np.array([x1, x2, x3, x4])
matrix_1 = np.array([[8,1,10,9],[2,-1,-6,7],[2,-6,5,-1],[3,5,2,10]])
matrix_2 = np.array([-6,-10,2,-7])

fx = np.dot(np.dot(x_array.T, matrix_1),x_array) + np.dot(x_array.T, matrix_2)
print(fx)

#px1 = fx.diff(x1)
#px2 = fx.diff(x2)
#px3 = fx.diff(x3)
#px4 = fx.diff(x4)

#grad1 = px1.subs({x1:1.01, x2:1, x3:1, x4:1})
#grad2 = px1.subs({x1:1, x2:1.01, x3:1, x4:1})
#grad3 = px1.subs({x1:1, x2:1, x3:1.01, x4:1})
#grad4 = px1.subs({x1:1, x2:1, x3:1, x4:1.01})

#fx1 = np.array([[grad1],[ori2],[ori3],[ori4]])
#fx2 = np.array([[ori1],[grad2],[ori3],[ori4]])
#fx3 = np.array([[ori1],[ori2],[grad3],[ori4]])
#fx4 = np.array([[ori1],[ori2],[ori3],[grad4]])

delta_x1 = (fx.subs({x1:1.01, x2:1, x3:1, x4:1})-fx.subs({x1:1, x2:1, x3:1, x4:1}))/0.01
delta_x2 = (fx.subs({x1:1, x2:1.01, x3:1, x4:1})-fx.subs({x1:1, x2:1, x3:1, x4:1}))/0.01
delta_x3 = (fx.subs({x1:1, x2:1, x3:1.01, x4:1})-fx.subs({x1:1, x2:1, x3:1, x4:1}))/0.01
delta_x4 = (fx.subs({x1:1, x2:1, x3:1, x4:1.01})-fx.subs({x1:1, x2:1, x3:1, x4:1}))/0.01

finite_diff = np.array([[delta_x1],[delta_x2],[delta_x3],[delta_x4]])
print(finite_diff)

res_x1 = np.array([[1],[1],[1],[1]]) - 0.01 * finite_diff
print(res_x1)
