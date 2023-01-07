import cv2
import numpy as np

def detect_lanes(image):
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]

    _, l_binary = cv2.threshold(l_channel, 145, 255, cv2.THRESH_BINARY)

    sobelx = np.abs(cv2.Sobel(l_binary, cv2.CV_64F, dx=1, dy=0, ksize=5))
    sobely = np.abs(cv2.Sobel(l_binary, cv2.CV_64F, dx=0, dy=1, ksize=5))
    sobel = np.sqrt(sobelx ** 2 + sobely ** 2)

    poly_points = np.float32([[0, 335], [240, 220], [400, 220], [640, 335]])
    image_points = np.float32([[0, 480], [0, 0], [640, 0], [640, 480]])

    roi = cv2.fillPoly(np.zeros_like(l_binary), np.int32([poly_points]), (255,255,255))
    cropped_image = cv2.bitwise_and(roi, l_binary)

    transformation_matrix = cv2.getPerspectiveTransform(poly_points, image_points)
    warped = cv2.warpPerspective(l_binary, transformation_matrix, (640, 480))

    return warped
