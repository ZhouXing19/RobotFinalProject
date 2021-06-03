import cv2
import matplotlib.pyplot as plt
import numpy as np

image = cv2.imread('./yellowdoor.png')

# Converting the image to hsv
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# define range of red color in HSV
lower_red = np.array([30, 164, 110])
upper_red = np.array([60, 255, 255])

# Threshold the HSV image using inRange function to get only red colors
mask = cv2.inRange(hsv, lower_red, upper_red)

plt.figure(figsize=[15, 15])
plt.subplot(211);
plt.imshow(image[:, :, ::-1]);
plt.title("Original Image", fontdict={'fontsize': 25});
plt.axis('off');
plt.subplot(212);
plt.imshow(mask, cmap='gray');
plt.title("Mask of red Color", fontdict={'fontsize': 25});
plt.axis('off');
plt.show()


