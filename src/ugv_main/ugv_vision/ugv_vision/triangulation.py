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

x = np.array([[4 - 0.08, -3, 0.5 - 0.08],
              [4 + 0.08, -3, 0.5 - 0.08],
              [4 + 0.08, -3, 0.5 + 0.08],
              [4 - 0.08, -3, 0.5 + 0.08]], dtype=np.float32)



u = np.array([[635.80334473, 493.31079102],
            [681.45776367, 489.84780884],
            [680.08850098, 436.08477783],
            [634.45465088, 438.38787842]])


for i in range(x.shape[0]):
    tmp = np.array([[x[i][0], x[i][1], x[i][2], 1, 0, 0, 0, 0, -u[i][0]*x[i][0], -u[i][0]*x[i][1], -u[i][0]*x[i][2], -u[i][0]],
                    [0, 0, 0, 0, x[i][0], x[i][1], x[i][2], 1, -u[i][1]*x[i][0], -u[i][1]*x[i][1], -u[i][1]*x[i][2], -u[i][1]]])
    
    if Q is None:
        Q = tmp
    else:
        Q = np.vstack((Q, tmp))


# for i in range(x.shape[0]):
#     tmp = np.array([[x[i][0], x[i][1], 1, 0, 0, 0, -u[i][0]*x[i][0], -u[i][0]*x[i][1], -u[i][0]],
#                     [0, 0, 0, x[i][0], x[i][1], 1, -u[i][1]*x[i][0], -u[i][1]*x[i][1], -u[i][1]]])
    
#     if Q is None:
#         Q = tmp
#     else:
#         Q = np.vstack((Q, tmp))



# # Since we want to solve M for Q @ M = 0, we need the singular value decomposition of Q
# U, S, Vt = np.linalg.svd(Q)
# M = Vt[-1].reshape(3, 4)

# # Decompose M to get intrinsic and extrinsic parameters
# K, R_T = np.linalg.qr(M)

# print(f"Intrinsic Matrix K:")
# print(K)
# print()
# print("Rotation-Translation Matrix R|T:")
# print(R_T)



# Since we want to solve M for Q @ M = 0, we need the singular value decomposition of Q
U, S, Vt = np.linalg.svd(Q)
M = Vt[-1].reshape(4, 3)

# Decompose M to get intrinsic and extrinsic parameters
R_T, K = np.linalg.qr(M)

print(f"Intrinsic Matrix K:")
print(K)
print()
print("Rotation-Translation Matrix R|T:")
print(R_T.T)
# print()
# print("Translation Vector:")
# print(R_T[:, -1] / R_T[2,2])


# real_pos = np.array([0, -3, 1.0]) # Real-world position of the robot
# proj_pos = K @ R_T.T @ real_pos

# print("Projected Position:")
# print(proj_pos/proj_pos[2])

# This is the center of the apriltag in robot coordinates with corner points deviating 8 cm from the x and y axes of the tag
# x = np.array([4, -3, 0.5])

# x = np.array([[4 + 0.08, -3, 0.5 - 0.08],
#               [4 - 0.08, -3, 0.5 - 0.08],
#               [4 - 0.08, -3, 0.5 + 0.08],
#               [4 + 0.08, -3, 0.5 + 0.08]], dtype=np.float32)

x = np.array([[0.08, -0.08, 0],
              [-0.08, -0.08, 0],
              [-0.08, 0.08, 0],
              [0.08, 0.08, 0]], dtype=np.float32)



u = np.array([[635.80334473, 493.31079102],
            [681.45776367, 489.84780884],
            [680.08850098, 436.08477783],
            [634.45465088, 438.38787842]])

_, rvec, tvec = cv2.solvePnP(x, u, intrinsic_params, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
print("PnP Result:")
print(rvec)
print(tvec)

r_mat = cv2.Rodrigues(rvec)[0]
extrinsic_mat = np.hstack((r_mat, tvec))
print("Extrinsic Matrix from PnP:")
print(extrinsic_mat)

dist = np.sqrt(extrinsic_mat[0, 3]**2 + extrinsic_mat[1, 3]**2)
print(f"Distance from PnP: {dist}")

norm = np.linalg.norm(extrinsic_mat[:, 3])
print(f"Norm of translation vector from PnP: {norm}")







# U, S, Vt = np.linalg.svd(Q)
# M = Vt[-1].reshape(4, 3)

# K, RT = np.linalg.qr(M)

# # print(f"Q matrix: {Q}")
# print(f"Shape of Q: {Q.shape}")

# print(f"K matrix: {K.T / K[3,2]}")
# print(f"RT matrix: {RT}")

# print(RT.T @ RT)  # Should be close to identity matrix
# print(intrinsic_params @ intrinsic_params.T)  # Should be close to identity matrix


# # t = RT[:, 2]
# # t = t / t[2]

# # print(t)

# # t = RT[:, 3]
# # t = t / t[2]

# # print(t)