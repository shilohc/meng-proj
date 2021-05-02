#!/usr/bin/env python3

import cv2
import sys

filename = sys.argv[1] + ".png"
img = cv2.imread(filename)
coords_x = int(sys.argv[2])
coords_y = int(sys.argv[3])

cv2.circle(img, center=(coords_x, coords_y), radius=5, color=(200, 100, 0), thickness=-1)
cv2.imwrite("test_out.png", img)
