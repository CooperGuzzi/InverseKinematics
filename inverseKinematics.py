# Cooper Guzzi 212442 and Maurice Trammell 216762

import numpy as np
import math
from arm_controller import ArmController

def getTransMatrix(posX, posY, posZ, psi, phi, theta):

    # default matrix
    matrix = np.eye(4)

    # set the values of the matrix using the user's input
    matrix[0][0] = math.cos(theta)*math.cos(phi)
    matrix[0][1] = (-math.sin(theta)*math.cos(psi)) + (math.cos(theta)*math.sin(phi)*math.sin(psi))
    matrix[0][2] = (math.sin(theta)*math.sin(psi)) + (math.cos(theta)*math.sin(phi)*math.cos(psi))
    matrix[0][3] = posX
    matrix[1][0] = math.sin(theta)*math.cos(phi)
    matrix[1][1] = (math.cos(theta)*math.cos(psi)) + (math.sin(theta)*math.sin(phi)*math.sin(psi))
    matrix[1][2] = (-math.cos(theta)*math.sin(psi)) + (math.sin(theta)*math.sin(phi)*math.cos(psi))
    matrix[1][3] = posY
    matrix[2][0] = -math.sin(phi)
    matrix[2][1] = math.cos(phi)*math.sin(psi)
    matrix[2][2] = math.cos(phi)*math.cos(psi)
    matrix[2][3] = posZ
    matrix[3][0] = 0
    matrix[3][1] = 0
    matrix[3][2] = 0
    matrix[3][3] = 1

    return matrix


def main():

    # get X, Y, Z, roll, pitch, and yaw at end of arm
    posX = input("Arm's X value? ")
    posY = input("Arm's Y value? ")
    posZ = input("Arm's Z value? ")
    psi = input("Arm's Roll? ")
    phi = input("Arm's Pitch? ")
    theta = input("Arm's Yaw? ")
        
    # Matrix = T
    matrix = getTransMatrix(posX, posY, posZ, psi, phi, theta)
    
    # We now need to use the values we got after multiplying all of the matrices together to solve for cos1, sin1, cos2, sin2, and cos3, sin3. 
    cos1 = matrix[1][1]
    sin1 = -matrix[0][1]
    cos23 = matrix[2][2]
    sin23 = -matrix[2][0]

    # Next we do jointAngle1 = arctan2(sin1, cos1)
    #            jointAngle2 = arctan2(sin2, cos2)
    #            jointAngle3 = 0 (default)
    #            jointAngle4 = arctan2(sin3, cos3)
    jointAngle1 = np.arctan2(sin1, cos1)
    jointAngle2 = math.acos((matrix[2][3]-0.077)/-0.126)+0.713
    sin2 = sin(jointAngle2)
    cos2 = cos(jointAngle2)
    jointAngle3 = np.arctan2(sin23, cos23) - np.arctan2(sin2, cos2)

    # Print joint angles to reach user's desired position
    print("Joint angles required to reach the desired position:") 
    print("Joint Angle 1:")
    print(str(jointAngle1)) 
    print("Joint Angle 2:") 
    print(str(jointAngle2)) 
    print("Joint Angle 3:") 
    print(str(jointAngle3)) 

    # Move the arm to the desired position
    ac = ArmController()
    ac.set_joints([jointAngle1, jointAngle2, 0, jointAngle3])
    # use to check our answer: print(ac.get_pose())

if __name__ == "__main__":
    main()
