import numpy as np
import math

dot_thresh = 0.999999
x_axis = np.array([1,0,0])
y_axis = np.array([0,1,0])
z_axis = np.array([0,0,1])

def updateQuat(q,gyro, dt):
    qGyro = np.array([0, gyro[0], gyro[1], gyro[2]])
    qDot = 0.5 * quatProd(q, qGyro)
    qUpdate = q + qDot*dt
    qUpdate = qUpdate / np.linalg.norm(qUpdate)
    return qUpdate

#assume v0 and v1 are normalized
def getQuat(v0, v1):
    cross = np.cross(v0, v1)
    dot = np.dot(v0, v1)
    if (dot < -dot_thresh):
        if (v0[0] <= v0[1] and v0[0] <= v0[2]):
            tmp = x_axis
        elif (v0[1] <= v0[0] and v0[1] <= v0[2]):
            tmp = y_axis
        else:
            tmp = z_axis
        orth_vec = np.cross(v0, tmp)
        q = np.array([0, orth_vec[0], orth_vec[1], orth_vec[2]])
    else:
        q = np.array([1 + dot, cross[0], cross[1], cross[2]])
    return q / np.linalg.norm(q)
    
def quatInv(q):
    qOut = np.zeros((4))
    qOut[:] = [q[0], -q[1], -q[2], -q[3]]
    return qOut

def quatProd(qA, qB):
    qNew = np.zeros((4))
    qNew[0] = qA[0]*qB[0] - qA[1]*qB[1] - qA[2]*qB[2] - qA[3]*qB[3]
    qNew[1] = qA[0]*qB[1] + qA[1]*qB[0] + qA[2]*qB[3] - qA[3]*qB[2]
    qNew[2] = qA[0]*qB[2] - qA[1]*qB[3] + qA[2]*qB[0] + qA[3]*qB[1]
    qNew[3] = qA[0]*qB[3] + qA[1]*qB[2] - qA[2]*qB[1] + qA[3]*qB[0]
    return qNew

def quatTransform(q, vIn):
    vInQuat = np.zeros((4))
    vInQuat[:] = [0, vIn[0], vIn[1], vIn[2]]
    vOut = quatProd(q, quatProd(vInQuat, quatInv(q)))
    return vOut[1:4]


def quat2eul(q):
    roll = math.atan2(q[0]*q[1] + q[2]*q[3], 1 - 2*(q[0]*q[0] + q[2]*q[2]))
    pitch = math.asin(2*(q[0]*q[2] - q[1]*q[3]))
    yaw = math.atan2(q[0]*q[3] + q[1]*q[2], 1 - 2*(q[2]*q[2] + q[3]*q[3]))
    
    return roll, pitch, yaw

def eul2quat(eul):
    roll = eul[0]
    pitch = eul[1]
    yaw = eul[2]
    
    q = np.zeros((4))
    q[0] = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
    q[1] = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
    q[2] = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)
    q[3] = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)
    
    return q
