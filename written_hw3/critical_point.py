from sympy import *
import numpy as np

x = Symbol('x')
fx = 0.25 * (x ** 4) + 2/3 * (x ** 3) - 5/2 * (x ** 2) - 6 * x + 1
gradient = fx.diff(x)
ans = solve(gradient, x)
print(ans)

hessian =