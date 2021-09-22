print("u sure?")

from os import MFD_HUGE_64KB
import numpy as np
import pybullet as p
import time
import pybullet_data

with open('pytest.txt') as f: #qrefnew(0.005) qscenario
    lines = f.readlines()
f.close()


N=372
M1=np.zeros(N*7)
i=0
for number in lines:
    M1[i]=float(number)
    i+=1

print(M1)

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

  
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])

planeId = p.loadURDF("plane.urdf") #ID:0
Sid=p.loadURDF("SURENA/surena5.urdf" )

p.setGravity(0,0,-9.81)

    ###################################################################################################################################################
p.setTimeStep(1./200.)


p.stepSimulation()

# for i in range(28):
#   p.setJointMotorControl2(1, i, p.POSITION_CONTROL, targetPosition=0)

joints=list(range(15,22)) #(14,21)
# for i in range(29):
#     print("INFO: ",p.getJointInfo(1,i))
joints2=list(range(28))
#print(joints2)

#M2=np.zeros([N,28])
#print(M2)

# for i in range(430):
#     M2[i][14:21]=M[i]

action=np.zeros([N,7])
obsevastion=np.zeros([N,7])

j=0
for i in range(N):

    time.sleep(1./200.)
    #print(M[i])

    p.setJointMotorControlArray(bodyUniqueId=1,
                                jointIndices=joints,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions = M1[j:j+7]) #targetPositions =action[0].numpy()) [0.25,0.25,0.25,0.25,0.25,0.25,0.25]
    # p.setJointMotorControlArray(bodyUniqueId=1,
    #                             jointIndices=list(range(15)),
    #                             controlMode=p.POSITION_CONTROL,
    #                             targetPositions =  [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    # p.setJointMotorControlArray(bodyUniqueId=1,
    #                             jointIndices=list(range(21,28)),
    #                             controlMode=p.POSITION_CONTROL,
    #                             targetPositions = [0,0,0,0,0,0,0])
    j+=7

    JointStates=p.getJointStates(1,joints) ##JPos,JVel,JF
    for ii in range(7):
        obsevastion[i][ii]=JointStates[ii][0]
    #action[i]= M1[j:j+7]
        
    
    JointStates=p.getJointStates(1,joints2) ##JPos,JVel,JF
    #print(JointStates)
    p.stepSimulation()



#p.disconnect()

#print(M.shape)

#M3=M.T

import matplotlib.pyplot as plt

m1=np.zeros(N)
m2=np.zeros(N)
m3=np.zeros(N)
m4=np.zeros(N)
m5=np.zeros(N)
m6=np.zeros(N)
m7=np.zeros(N)

i=0
ii=0
while i<N*7-6:
    m1[ii]=M1[i]
    m2[ii]=M1[i+1]
    m3[ii]=M1[i+2]
    m4[ii]=M1[i+3]
    m5[ii]=M1[i+4]
    m6[ii]=M1[i+5]
    m7[ii]=M1[i+6]
    i+=7
    ii+=1
    


plt.figure()
plt.plot(obsevastion)
# plt.show()


plt.figure()
plt.plot(m1)
# plt.figure()
plt.plot(m2)
# plt.figure()
plt.plot(m3)
# plt.figure()
plt.plot(m4)
# plt.figure()
plt.plot(m5)
# plt.figure()
plt.plot(m6)
# plt.figure()
plt.plot(m7)
plt.legend([1,2,3,4,5,6,7])
plt.show() 

# plt.figure()
# plt.plot(M1[:430],"*")
# plt.show()


# for i in range(7):

#     plt.figure()
#     plt.plot(M3[i].T,'*')
#     #plt.legend([1,2,3,4,5,6,7])
#     plt.ylabel('some numbers')


# plt.show()