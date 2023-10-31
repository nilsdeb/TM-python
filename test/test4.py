# pour les vecteurs et les quaternions
import numpy as np
from numpy import linalg as LA



# pour les maths
import math as m


def rotate_vector_with_angle_and_axis(axis, vector_to_rotate):


    # Ensure that the axis is a unit vector



    norme = LA.norm(axis)
    print(norme)
    angle = norme
    axis2 = axis/norme



    ux, uy, uz = axis2
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    one_minus_cos_theta = 1 - cos_theta

    # Construct the rotation matrix
    R = np.array([
        [cos_theta + ux**2 * one_minus_cos_theta, ux * uy * one_minus_cos_theta - uz * sin_theta, ux * uz * one_minus_cos_theta + uy * sin_theta],
        [uy * ux * one_minus_cos_theta + uz * sin_theta, cos_theta + uy**2 * one_minus_cos_theta, uy * uz * one_minus_cos_theta - ux * sin_theta],
        [uz * ux * one_minus_cos_theta - uy * sin_theta, uz * uy * one_minus_cos_theta + ux * sin_theta, cos_theta + uz**2 * one_minus_cos_theta]
    ])

    # Rotate the vector using matrix-vector multiplication
    rotated_vector = np.dot(R, vector_to_rotate)

    return rotated_vector




vec = np.array([1,0,0])
vec1 = np.array([1,6,0])
vecangle = np.array([m.pi,m.pi,m.pi])
a = rotate_vector_with_angle_and_axis(vecangle,vec1)

print(a,LA.norm(vec1),LA.norm(a))
