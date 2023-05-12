import numpy as np
import cv2
import math

def Rot(yaw):
    yaw = math.radians(yaw)
    res = np.matrix([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    return res

detections=list()

img_res=np.array((640,480))
img = cv2.imread("../resource/map.png")
img = cv2.resize(img, dsize=img_res, interpolation=cv2.INTER_AREA)
cv2.imshow("img",img)
print(img.shape)

detection=np.array((245,23))
detection=detection-img_res/2
detection[1]=-detection[1]

HFOV=math.radians(62.2)
VFOV=math.radians(48.8)

drone_pos=np.array((10,10))
drone_yaw=34
drone_amplitude=18

cam_range=(math.tan(HFOV)*drone_amplitude,math.tan(VFOV)*drone_amplitude)
print(cam_range)

target_pos_rel=np.multiply(np.divide(detection, img_res), cam_range)

print(target_pos_rel)

detections.append(drone_pos+np.matmul(Rot(drone_yaw), target_pos_rel))


# cv2.waitKey(0)
# cv2.destroyAllWindows()