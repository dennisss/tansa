import numpy as np
import cv2
from matplotlib import pyplot as plt

full = cv2.imread('full.jpg')
gray = cv2.cvtColor(full, cv2.COLOR_BGR2GRAY)
width = gray.shape[1] / 2

imgL = gray[:, :width]
imgR = gray[:, width:]

stereo = cv2.StereoBM_create(numDisparities=32, blockSize=5)
disparity = stereo.compute(imgL,imgR)
plt.imshow(disparity,'gray')
plt.show()
