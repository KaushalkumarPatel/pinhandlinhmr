from moviepy.editor import VideoFileClip
from IPython.display import HTML
import numpy as np
import cv2


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

def pipeline(image):
    img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    img_blur = cv2.medianBlur(img_hsv, 9)
    img_blur = cv2.GaussianBlur(img_blur, (11, 11), 0)
    img_blur = cv2.blur(img_blur,(37,37))

    img_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    img_gray_blur = cv2.medianBlur(img_gray, 9)
    img_gray_blur = cv2.GaussianBlur(img_gray_blur, (11, 11), 0)
    img_gray_blur = cv2.blur(img_gray_blur,(37,37))
    ret,thresh = cv2.threshold(img_gray_blur,110,255,cv2.THRESH_BINARY)

    if np.average(thresh) > 110:
        mask = cv2.inRange(img_blur, (18,55,160), (24,90,235))
    else:
        mask = cv2.inRange(img_blur, (24,20,105), (45,70,195))

    mask = cv2.erode(mask, None, iterations=5)
    mask = cv2.dilate(mask, None, iterations=5)

    img_hpf = mask - cv2.GaussianBlur(mask, (21, 21), 3)+127
    img_canny = cv2.Canny(img_hpf, 200, 100)

    lines = cv2.HoughLinesP(
        img_canny,
        rho=6,
        theta=np.pi / 60,
        threshold=160,
        lines=np.array([]),
        minLineLength=100,
        maxLineGap=20
    )

    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []

    try:
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
    except:
        pass

    min_y = 0
    max_y = image.shape[0]

    try:
        poly_left = np.poly1d(np.polyfit(
            left_line_y,
            left_line_x,
            deg=1
        ))
        left_x_start = int(poly_left(max_y))
        left_x_end = int(poly_left(min_y))
    except:
        left_x_start = 0
        left_x_end = 0

    try:
        poly_right = np.poly1d(np.polyfit(
            right_line_y,
            right_line_x,
            deg=1
        ))
        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))
    except:
        right_x_start = 0
        right_x_end = 0

    line_image = draw_lines(
        image,
        [[
            [left_x_start, max_y, left_x_end, int(min_y)],
            [right_x_start, max_y, right_x_end, int(min_y)],
        ]],
        thickness=5,
    )

    return line_image


white_output = 'solidWhiteRight_output.mp4'
clip1 = VideoFileClip("/home/marram/catkin_ws/src/pinhandling_companion/pinhandling_cam/res/PXL_20230628_101002905.mp4")
white_clip = clip1.fl_image(pipeline)
white_clip.write_videofile(white_output, audio=False)