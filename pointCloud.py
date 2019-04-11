import numpy as np

from numpy import sin, cos

import csv

# t represents theta, l is the length of upper arm in shoulder coordinate
def locElbow(t1, t2, l):
    t1 = deg2rad(t1)
    t2 = deg2rad(t2)
    x = np.cos(t2)*np.sin(t1)*l
    y = np.sin(t2)*l
    z = -np.cos(t1)*np.cos(t2)*l
    return np.array([x, y, z])

# t represents theta, l is the length of lower arm
def locElhow2wraist(t3, t4, l):
    t3 = deg2rad(t3)
    t4 = deg2rad(t4)
    x = np.cos(t3)*np.cos(t4)*l
    y = np.sin(t3)*np.cos(t4)*l
    z = np.sin(t4)*l
    return np.array([x, y, z])

def deg2rad(angle):
    return angle*np.pi/180

# set intervals of the angles
interval = 1.
# set length of upper arm
lupper = 1.
llower = 1.

# define angles and their range with interval specified above
theta1 = np.arange(-60, 180+interval, interval)
theta2 = np.arange(-40, 120+interval, interval)
theta3 = np.arange(-30, 120+interval, interval)
theta4 = np.arange(0, 150+interval, interval)
theta5 = np.arange(0, 180+interval, interval)

#theta1 = deg2rad(np.arange(-60, 180+interval, interval))
#theta2 = deg2rad(np.arange(-40, 120+interval, interval))
#theta3 = deg2rad(np.arange(-30, 120+interval, interval))
#theta4 = deg2rad(np.arange(0, 150+interval, interval))
#theta5 = deg2rad(np.arange(0, 180+interval, interval))

print ('start computing elbow possible locations')
# compute the whole location of elbow point cloud
ElbowLoc = []
ElbowLocDict = {}
for angle1 in theta1:
    for angle2 in theta2:
        ElbowLoc.append([angle1, angle2, locElbow(angle1, angle2, lupper)])
        ElbowLocDict[(angle1, angle2)] = locElbow(angle1, angle2, lupper)

print ('elbow location complete')

# wrist location in elbow frame
RelativeWristLoc = []
for angle3 in theta3:
    for angle4 in theta4:
        RelativeWristLoc.append([angle3, angle4, locElhow2wraist(angle3, angle4, llower)])

# wrist location in global frame
WristLoc = []
WristLocDict = {}
for i in range(len(ElbowLoc)):
    for j in range(len(RelativeWristLoc)):
        Loc = ElbowLoc[i][2]+RelativeWristLoc[j][2]
        WristLoc.append([ElbowLoc[i][0], ElbowLoc[i][1], RelativeWristLoc[j][0], RelativeWristLoc[j][1], Loc])
        WristLocDict[(ElbowLoc[i][0], ElbowLoc[i][1], RelativeWristLoc[j][0], RelativeWristLoc[j][1])] = Loc

print('wrist location complete')


# compute rotation of the watch
def watchX(t1, t2, t3, t4, t5):
    x = cos(t1)*cos(t3)*cos(t4)-sin(t1)*sin(t2)*sin(t3)*cos(t4)-sin(t1)*cos(t2)*sin(t4)
    y = cos(t2)*sin(t3)*cos(t4)-sin(t2)*sin(t4)
    z = sin(t1)*cos(t3)*cos(t4)+cos(t1)*sin(t2)*sin(t3)*cos(t4)+cos(t1)*cos(t2)*sin(t4)
    return np.array([x, y, z])


def watchY(t1, t2, t3, t4, t5):
    x = cos(t5)*(-cos(t1)*sin(t3)-sin(t1)*sin(t2)*cos(t3))+sin(t5)*(-cos(t1)*cos(t3)*sin(t4)+sin(t1)*sin(t2)*sin(t3)*sin(t4)-sin(t1)*cos(t2)*cos(t4))
    y = cos(t5)*(cos(t2)*cos(t3))+sin(t5)*(-cos(t2)*sin(t3)*sin(t4)-cos(t4)*sin(t2))
    z = cos(t5)*(-sin(t1)*sin(t3)+cos(t1)*sin(t2)*cos(t3))+sin(t5)*(-sin(t1)*cos(t3)*sin(t4)-cos(t1)*sin(t2)*sin(t3)*sin(t4)+cos(t1)*cos(t2)*cos(t4))
    return np.array([x, y, z])


def watchZ(t1, t2, t3, t4, t5):
    x = -sin(t5)*(-cos(t1)*sin(t3)-sin(t1)*sin(t2)*cos(t3))+cos(t5)*(-cos(t1)*cos(t3)*sin(t4)+sin(t1)*sin(t2)*sin(t3)*sin(t4)-sin(t1)*cos(t2)*cos(t4))
    y = -sin(t5)*(cos(t2)*cos(t3))+cos(t5)*(-cos(t2)*sin(t3)*sin(t4)-cos(t4)*sin(t2))
    z = cos(t5)*(-sin(t1)*sin(t3)+cos(t1)*sin(t2)*cos(t3))+cos(t5)*(-sin(t1)*cos(t3)*sin(t4)-cos(t1)*sin(t2)*sin(t3)*sin(t4)+cos(t1)*cos(t2)*cos(t4))
    return np.array([x, y, z])


def watchXdirection(elbowloc, wristloc):
    x = wristloc[0] - elbowloc[0]
    y = wristloc[1] - elbowloc[1]
    z = wristloc[2] - elbowloc[2]
    return (x, y, z)



# gives rotation cloud of the watch in the form of [theta1, 2, 3, 4, 5, pointing vector of respective watch axes]
RotateWatch_x = []
RotateWatch_y = []
RotateWatch_z = []

count = 0.
totallength = len(theta1)
for i in range(len(WristLoc)):
    eloc = ElbowLocDict[(WristLoc[i][0], WristLoc[i][1])]
    wloc = WristLoc[i][4]
    xdir = watchXdirection(eloc, wloc)
    RotateWatch_x.append([WristLoc[i][0], WristLoc[i][1], WristLoc[i][2], WristLoc[i][3], xdir])

print(RotateWatch_x[1])

print('watch orientation complete')

# establish point cloud: {rotation of the watch: all possible locations of elbow and wrist}
ElbowPointCloud = {}
lofPointCloud = len(RotateWatch_x)
for i in range(lofPointCloud):
        ElbowPointCloud[RotateWatch_x[i][4]] = ElbowLocDict[(RotateWatch_x[i][0], RotateWatch_x[i][1])]

print('complete creating elbow point cloud')

WristPointCloud = {}
for i in range(lofPointCloud):
    WristPointCloud[RotateWatch_x[i][4]] = WristLocDict[(RotateWatch_x[i][0], RotateWatch_x[i][1], RotateWatch_x[i][2], RotateWatch_x[i][3])]

print('complete creating wrist point cloud')


ElbowPointCloudData = []
for keys in ElbowPointCloud:
    ElbowPointCloudData.append([keys, ElbowPointCloud[keys]])

for i in range(len(ElbowPointCloudData)):
    ElbowPointCloudData[i].insert(0, i)

WristPointCloudData = []
for keys in WristPointCloud:
    WristPointCloudData.append([keys, WristPointCloud[keys]])

for i in range(len(WristPointCloudData)):
    WristPointCloudData[i].insert(0, i)


with open('ElbowPointCloud.csv', mode='w') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerows(ElbowPointCloudData)

csv_file.close()

with open('WristPointCloud.csv', mode='w') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerows(WristPointCloudData)

csv_file.close()









