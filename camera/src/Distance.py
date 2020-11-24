import math
import cv2
import numpy as np
import pyzed.sl as sl
import sys
import bouydetection as bd





def distance(zed):
    #zed = sl.Camera()

    '''
    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)
    '''

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100

    # Capture 150 images and depth, then stop
    i = 0

    image_zed = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    point_cloud_mid = sl.Mat()


    '''
    image_size = zed.get_camera_information().camera_resolution
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    depth = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    point_cloud = sl.Mat()
    '''#image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width,
                       #zed.get_camera_information().camera_information.height, sl.MAT_TYPE.U8_C4)

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75, 4.0, 0))
    tr_np = mirror_ref.m



    while True:
        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            # Retrieve depth map. Depth is aligned on the left image
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            #x = round(image.get_width() / 2)
            #y = round(image.get_height() / 2)
            # main.printimage('raw', image)

            image = image_zed.get_data()
            #image = imutils.resize(image, width=600)

            hsvimg = bd.imgtohsv(image)
            # printimage('hsv', hsvimg)
            mask = bd.colormask(hsvimg)
            # printimage('mask', mask)
            result = cv2.bitwise_and(image, image, mask=mask)
            # printimage('result', result)
            x, y, img, og_img= bd.shapemask(result, image)


            h2 = round(result.shape[0] / 2)
            w2 = round(result.shape[1] / 2)
            #print(h2)
            #print(w2)

            '''
            err, point_cloud_value_mid = point_cloud_mid.get_value(w2, h2)
            distance_mid = math.sqrt(point_cloud_value_mid[0] * point_cloud_value_mid[0] +
                                 point_cloud_value_mid[1] * point_cloud_value_mid[1] +
                                 point_cloud_value_mid[2] * point_cloud_value_mid[2])
            '''


            err, point_cloud_value = point_cloud.get_value(x, y)
            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])



            point_cloud_np = point_cloud.get_data()
            point_cloud_np.dot(tr_np)

            ang = angle(point_cloud,distance,x,w2,point_cloud_mid)


            if not np.isnan(distance) and not np.isinf(distance):
                #print("Distance to Camera at ({}, {}) (bouy location): {:1.3} m".format(x, y, distance), end="\r")
                cv2.putText(og_img,"{:.2f}m".format(round(distance/1000, 2)) , (x - 20, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(og_img, str(round(ang)), (x-20, y - 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 2)
                # Increment the loop
                #i = i + 1
            cv2.imshow("slut", og_img)
            cv2.waitKey(1)
            '''
            else:
                print("Can't estimate distance at this position.")
                print("Your camera is probably too close to the scene, please move it backwards.\n")
            '''
            sys.stdout.flush()

    #return distance
    # Close the camera
    #zed.close()