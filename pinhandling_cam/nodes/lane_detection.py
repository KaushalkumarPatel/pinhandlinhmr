import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2

##
##  Load image
##
# img = mpimg.imread('/home/marram/catkin_ws/src/pinhandling_companion/pinhandling_camres/PXL_20230628_101427010.jpg')
# img = mpimg.imread('/home/marram/catkin_ws/src/pinhandling_companion/pinhandling_cam/res/PXL_20230628_101049345.jpg')
img = mpimg.imread('/home/marram/catkin_ws/src/pinhandling_companion/pinhandling_cam/res/PXL_20230628_101054048.jpg')
# img = mpimg.imread('/home/marram/catkin_ws/src/pinhandling_companion/pinhandling_cam/res/PXL_20230628_101057421.jpg')

plt.figure()
plt.imshow(img)

##
##  Convert to HSV color space
##
img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
plt.figure()
plt.imshow(img_hsv)

##
##  Filter salt-and-pepper noise
##
img_blur = cv2.medianBlur(img_hsv, 9)
plt.figure()
plt.imshow(img_blur)

##
##  Additional gaussian blur
##
img_blur = cv2.GaussianBlur(img_blur, (11, 11), 0)
plt.figure()
plt.imshow(img_blur)

##
##  Additional average blur
##
img_blur = cv2.blur(img_blur,(37,37))
plt.figure()
plt.imshow(img_blur)

##
##  Determine the average threshold of the gray-scale image
##
img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
img_gray_blur = cv2.medianBlur(img_gray, 9)
img_gray_blur = cv2.GaussianBlur(img_gray_blur, (11, 11), 0)
img_gray_blur = cv2.blur(img_gray_blur,(37,37))
ret,thresh = cv2.threshold(img_gray_blur,110,255,cv2.THRESH_BINARY)
plt.figure()
plt.imshow(thresh, 'gray')

##
##  Choose a mask depending on the threshold
##
if np.average(thresh) > 110:
    mask = cv2.inRange(img_blur, (18,55,160), (24,90,235))
else:
    mask = cv2.inRange(img_blur, (24,20,105), (45,70,195))
plt.figure()
plt.imshow(mask)

##
##  Mask cleanup
##
mask = cv2.erode(mask, None, iterations=5)
mask = cv2.dilate(mask, None, iterations=5)
plt.figure()
plt.imshow(mask)

##
##  High pass filter the image to re-create sharp edges
##
img_hpf = mask - cv2.GaussianBlur(mask, (21, 21), 3) + 127
plt.figure()
plt.imshow(img_hpf)

##
##  Edge detection with Canny algorithm
##
img_canny = cv2.Canny(img_hpf, 200, 100)
plt.figure()
plt.imshow(img_canny)

##
## Calculate Hough lines
##
lines = cv2.HoughLinesP(
    img_canny,
    rho=6,
    theta=np.pi / 60,
    threshold=160,
    lines=np.array([]),
    minLineLength=100,
    maxLineGap=20
)

def draw_lines(img, lines, color=[255,255,255], thickness=5):
    if lines is None:
        return
    img = np.copy(img)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
    return img

line_image = draw_lines(img, lines)
plt.figure()
plt.imshow(line_image)


left_line_x = []
left_line_y = []
right_line_x = []
right_line_y = []
for line in lines:
    for x1, y1, x2, y2 in line:
        slope = (y2 - y1) / (x2 - x1)
        if abs(slope) < 0.5:
            continue
        if slope < 0:
            left_line_x.extend([x1, x2])
            left_line_y.extend([y1, y2])
        else:
            right_line_x.extend([x1, x2])
            right_line_y.extend([y1, y2])

min_y = 0
max_y = img.shape[0]

poly_left = np.poly1d(np.polyfit(left_line_y, left_line_x, deg=1))
left_x_start = int(poly_left(max_y))
left_x_end = int(poly_left(min_y))

poly_right = np.poly1d(np.polyfit(right_line_y, right_line_x, deg=1))
right_x_start = int(poly_right(max_y))
right_x_end = int(poly_right(min_y))

line_image = draw_lines(img, [[[left_x_start, max_y, left_x_end, int(min_y)], [right_x_start, max_y, right_x_end, int(min_y)]]], thickness=5)
plt.figure()
plt.imshow(line_image)


plt.show()