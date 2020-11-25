import imutils
import math
import cv2
import numpy as np
import pyzed.sl as sl


def angle(d, x, mx):
    #print("avstånd b", x-mx)
    b = x-mx

    r = math.acos(b/d)
    d_angle = r*(180/math.pi)

    return d_angle


def grab_frame(zed, runtime_parameters):
    # A new image is available if grab() returns SUCCESS
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        image_zed = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()
        # Retrieve left image
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        # Retrieve depth map. Depth is aligned on the left image
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        # Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        image = image_zed.get_data()
        return image, point_cloud


def imgtohsv(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return hsv


def detect_contour(image):
    hsvimg = imgtohsv(image)
    # printimage('hsv', hsvimg)
    mask = colormask(hsvimg)
    # printimage('mask', mask)
    result = cv2.bitwise_and(image, image, mask=mask)
    # printimage('result', result)
    x, y = shapemask(result)
    return x, y


def colormask(hsvimage):
    light_orange = (0, 150, 95)
    dark_orange = (11, 255, 255)
    maskedimage = cv2.inRange(hsvimage, light_orange, dark_orange)

    return maskedimage


def shapemask(image):
    cX, cY = 0, 0

    # image = imutils.resize(image, width=600)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    thresh = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY)[1]

    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts) > 0:
        cnts = max(cnts, key=cv2.contourArea)
        # compute the center of the contour
        M = cv2.moments(cnts)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
    return cX, cY


def detectshape(c):
    # initialize the shape name and approximate the contour
    shape = "unidentified"
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)

    if len(approx) == 3:
        shape = "triangle"
    # if the shape has 4 vertices, it is either a square or
    # a rectangle
    elif len(approx) == 4:
        # compute the bounding box of the contour and use the
        # bounding box to compute the aspect ratio
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)
        # a square will have an aspect ratio that is approximately
        # equal to one, otherwise, the shape is a rectangle
        shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
    # if the shape is a pentagon, it will have 5 vertices
    elif len(approx) == 5:
        shape = "pentagon"
    # otherwise, we assume the shape is a circle
    else:
        shape = "circle"

        # return the name of the shape
    if shape == "circle":
        return shape
    else:
        return 0


def startzedCamera():
    print("Running...")
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_minimum_distance = 0.15
    init_params.depth_maximum_distance = 40

    '''
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720
    '''
    init = sl.InitParameters()

    if not zed.is_opened():
        print("Opening ZED Camera...")
    status = zed.open(init)
    return zed, status


"""
def camgrabzed(cam):
    mat = sl.Mat()
    runtime = sl.RuntimeParameters()

    err = cam.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS:
        cam.retrieve_image(mat, sl.VIEW.LEFT)
        key = cv2.waitKey(5)

    return mat.get_data()
"""


def readimage(path):
    return cv2.imread(path, 1)


def startcap():
    cap = cv2.VideoCapture(0)
    return cap


def camgrabopencv(cap):
    ret, frame = cap.read()
    return frame


def imcrop(img, bbox):
    x1, y1, x2, y2 = bbox
    if x1 < 0 or y1 < 0 or x2 > img.shape[1] or y2 > img.shape[0]:
        img, x1, x2, y1, y2 = pad_img_to_fit_bbox(img, x1, x2, y1, y2)
    return img[y1:y2, x1:x2, :]


def pad_img_to_fit_bbox(img, x1, x2, y1, y2):
    img = np.pad(img, ((np.abs(np.minimum(0, y1)), np.maximum(y2 - img.shape[0], 0)),
                       (np.abs(np.minimum(0, x1)), np.maximum(x2 - img.shape[1], 0)), (0, 0)), mode="constant")
    y1 += np.abs(np.minimum(0, y1))
    y2 += np.abs(np.minimum(0, y1))
    x1 += np.abs(np.minimum(0, x1))
    x2 += np.abs(np.minimum(0, x1))
    return img, x1, x2, y1, y2
