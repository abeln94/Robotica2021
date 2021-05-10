import cv2

img = cv2.imread("mask.png")
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

sum = 0
for i in range(80):
    for j in range(320):
        if (img[i][j] == 255): sum = sum + 1

print(sum / 320 / 80)
