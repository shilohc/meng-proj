from ast import literal_eval
import cv2

image = cv2.imread("test_out.png")
with open("out.txt", 'r') as f:
    for line in f.readlines():
        if line == "OOB\n":
            continue
        x, y = literal_eval(line)
        image = cv2.circle(image, (x, y), 1, (200, 0, 200), -1)

cv2.imwrite("test_out_with_points.png", image)

