
from sympy import symbols, init_printing, Matrix
from sympy import sin as s
from sympy import cos as c
from sympy import Abs
import numpy as np

init_printing(use_unicode=True)

q1, q2, q3 = symbols('theta_1 theta_2 d_3')

l1 = 100
l2 = 50
l3 = 150

# let's implement the direct kinematics symbolically
DK = Matrix([
[c(q1)*c(q2), -s(q1), c(q1)*s(q2), (l2 + l3 + q3)*c(q1)*s(q2)],
[c(q2)*s(q1), c(q1), s(q1)*s(q2), (l2 + l3 + q3)*s(q1)*s(q2)],
[-s(q2), 0, c(q2), l1 + (l2 + l3 + q3)*c(q2)],
[0, 0, 0, 1]
])

# reshape to column vector A = [a11, a21, a31, ..., a34]
A = DK[0:3, 0:4] # crop last row

A = A.transpose().reshape(12,1)

Q = Matrix([q1, q2, q3])

J = A.jacobian(Q)
print(J)
q_init = Matrix([[np.deg2rad(40), np.deg2rad(40), 20]]).transpose()
A_init = A.evalf(subs={'theta_1': q_init[0],'theta_2': q_init[1],'d_3': q_init[2]})

A_final = Matrix([
    0.5792, 0.5792, -0.5736,
    -0.7071, 0.7071, 0,
    0.4056, 0.4056, 0.8192,
    93.2834, 93.2834, 288.4050
])


def incremental_ik(A, q_current, A_current, A_final):
    delta_A = (A_final - A_current)
    while max(Abs(delta_A)) > 0.01:
        J_q = J.evalf(subs={'theta_1': q_current[0],'theta_2': q_current[1], 'd_3': q_current[2]})
        # use pseudoinverse to solve over-determined problem
        # delta_A/10 is our linear interpolation between current and final pose
        delta_q = J_q.pinv() @ delta_A/10 # @ is matrix product
        q_current = q_current + delta_q
        A_current = A.evalf(subs={'theta_1': q_current[0],'theta_2': q_current[1], 'd_3': q_current[2]})
        delta_A = (A_final - A_current)
    return q_current


q_final = incremental_ik(A, q_init, A_init, A_final)

A_final2 = A.evalf(subs={'theta_1': q_final[0],'theta_2': q_final[1],'d_3': q_final[2]})
print(A_final2.reshape(4,3))