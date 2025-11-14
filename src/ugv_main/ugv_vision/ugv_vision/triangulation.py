import cv2
import numpy as np

x = np.array([[0, -3, 0.36],
              [-1.86, -3, 0.47]], dtype=np.float32)

u = np.array([[291.00497124, 448.90175765],
              [960.35098795, 381.46194432]], dtype=np.float32)

real_pos = np.array([-0.9, -2.05]) # Real-world position of the robot

intrinsic_params = np.array([[289.11451,   0.     , 347.23664],
                             [0.     , 289.75319, 235.67429],
                             [0.     ,   0.     ,   1.     ]])

distances = np.array([1.02, 1.53])

Q = None


for i in range(x.shape[0]):
    tmp = np.array([[x[i][0], x[i][1], x[i][2], 1, 0, 0, 0, 0, -u[i][0]*x[i][0], -u[i][0]*x[i][1], -u[i][0]*x[i][2], -u[i][0]],
                    [0, 0, 0, 0, x[i][0], x[i][1], x[i][2], 1, -u[i][1]*x[i][0], -u[i][1]*x[i][1], -u[i][1]*x[i][2], -u[i][1]]])
    
    if Q is None:
        Q = tmp
    else:
        Q = np.vstack((Q, tmp))

U, S, Vt = np.linalg.svd(Q)
M = Vt[-1].reshape(3, 4)

K, RT = np.linalg.qr(M)

# print(f"Q matrix: {Q}")
print(f"Shape of Q: {Q.shape}")

print(f"K matrix: {K}")
print(f"RT matrix: {RT}")

print(RT @ RT.T)  # Should be close to identity matrix
print(intrinsic_params @ intrinsic_params.T)  # Should be close to identity matrix


t = RT[:, 2]
t = t / t[2]

print(t)

t = RT[:, 3]
t = t / t[2]

print(t)