import numpy as np

class Frame():
    def __init__(self,obs_pose,drone_pose):
        self.obs_pose = obs_pose
        self.drone_pose = drone_pose
        self.length = len(obs_pose)
        self.index = 0

    def getNext(self):
        print("Processing Frame:", self.index)
        x_drone = np.empty((0,1))
        y_drone = np.empty((0,1))
        z_drone = np.empty((0,1))
        x = np.empty((0,1))
        y = np.empty((0,1))
        z = np.empty((0,1))
        obs1 = self.obs_pose[self.index][0]
        obs2 = self.obs_pose[self.index][1]
        obs3 = self.obs_pose[self.index][2]
        obs4 = self.obs_pose[self.index][3]
        # print(obs1)
        drone = self.drone_pose[self.index]
        # x
        x_drone = np.append(x_drone, np.array([[drone[0]]]), axis=0)
        x = np.append(x, np.array([[obs1[0]]]),axis = 0)
        x = np.append(x, np.array([[obs2[0]]]),axis = 0)
        x = np.append(x, np.array([[obs3[0]]]), axis=0)
        x = np.append(x, np.array([[obs4[0]]]), axis=0)
        x = np.append(x, np.array([[obs1[0]]]), axis=0)
        x = np.append(x, np.array([[obs2[0]]]), axis=0)
        x = np.append(x, np.array([[obs3[0]]]), axis=0)
        x = np.append(x, np.array([[obs4[0]]]), axis=0)
        # y
        y_drone = np.append(y_drone, np.array([[drone[1]]]),axis=0)
        y = np.append(y, np.array([[obs1[1]]]), axis=0)
        y = np.append(y, np.array([[obs2[1]]]), axis=0)
        y = np.append(y, np.array([[obs3[1]]]), axis=0)
        y = np.append(y, np.array([[obs4[1]]]), axis=0)
        y = np.append(y, np.array([[obs1[1]]]), axis=0)
        y = np.append(y, np.array([[obs2[1]]]), axis=0)
        y = np.append(y, np.array([[obs3[1]]]), axis=0)
        y = np.append(y, np.array([[obs4[1]]]), axis=0)
        # z
        z_drone = np.append(z_drone, np.array([[drone[2]]]),axis = 0)
        z = np.append(z, np.array([[obs1[2]]]), axis = 0)
        z = np.append(z, np.array([[obs2[2]]]), axis=0)
        z = np.append(z, np.array([[obs3[2]]]), axis=0)
        z = np.append(z, np.array([[obs4[2]]]), axis=0)
        z = np.append(z, np.array([[0]]), axis=0)
        z = np.append(z, np.array([[0]]), axis=0)
        z = np.append(z, np.array([[0]]), axis=0)
        z = np.append(z, np.array([[0]]), axis=0)
        if(self.index < self.length):
            self.index += 1
        return x,y,z,x_drone,y_drone,z_drone