import numpy as np
import argparse
import cv2
from PIL import Image
"""ap = argparse.ArgumentParser()
ap.add_argument("-i", "-Pictures/front-0_scence_7", help = "path to the image")
args = vars(ap.parse_args())
# load the image
image = cv2.imread(args["image"])

lower = np.array([105,105,105], dtype = "uint8")
upper = np.array([128,128,128], dtype = "uint8")


mask = cv2.inRange(image, lower, upper)
output = cv2.bitwise_and(image, image, mask = mask)
print(output)
# show the images
cv2.imshow("images", np.hstack([image, output]))
cv2.waitKey(0)"""

img = cv2.imread("front-0_scene_7.png")
##resized_img = img.resize(128,72)
print("made it BITCH")
##resized_img = cv2.resize(img,(128, 72))

hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

img.resize(256,144) ## width , height


lower = np.array([0,0,0], dtype = "uint8")
upper = np.array([192,192,192], dtype = "uint8")

mask = cv2.inRange(hsv, lower, upper)

cv2.imshow("Image",img)
cv2.imshow("Mask",mask)
print(mask)
print(img)
print("break")
print(np.argwhere(mask)[5])
cv2.waitKey(0)
