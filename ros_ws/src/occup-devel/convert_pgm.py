import cv2
import numpy as np
import matplotlib
from matplotlib import pyplot as plt 
import os 

in_dir = 'map1.jpg'
out_dir = '../occup-devel/'

map_img = cv2.imread(in_dir)
height, width, channel = map_img.shape
gray_map = cv2.cvtColor(map_img, cv2.COLOR_BGR2GRAY)

imgage_bw = undefined

ret, thresh = cv2.threshold(gray_map, imgage_bw, 200, 255.0, THRESH_BINARY) 

img_grid = imgage_bw/255

os.chdir(out_dir)

cv2.imwrite("occtest.pgm", img_grid)

with open(out_dir, 'w') as pgm_file: 
	pgm_file.write("P2\n{} {}\n255\n".format(width, height))

file.close()







