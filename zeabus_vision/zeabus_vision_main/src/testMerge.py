#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
imgL = None
imgR = None


def findKeyPoints(ref, fim):

    print "Fine Tuning using SIFT-based image-to-image registration"
    print "Find keys points between images"
    size_width = 1000
    margins = 100
    # sf = cv2.SIFT()
    sf = cv2.xfeatures2d.SIFT_create()
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)  # or pass empty dictionary
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    data_h, data_w = ref.shape
    maching_pairs = []
    distances = []
    for k in range(0, data_h, size_width):
        for m in range(0, data_w, size_width):
            str_row = max(0, k - margins)
            str_col = max(0, m - margins)
            stp_row = min(data_h, k + size_width + margins)
            stp_col = min(data_w, m + size_width + margins)
            print "working on (%d, %d) -> (%d,%d) " % (str_row, str_col, stp_row, stp_col)
            data_k = ref[str_row:stp_row, str_col:stp_col]
            data_k = (data_k > 0) * (data_k < 255) * \
                data_k + 255 * (data_k >= 255)
            data_k = np.round(data_k).astype('uint8')
            keys_ref, des_ref = sf.detectAndCompute(data_k, None)
            data_k = fim[str_row:stp_row, str_col:stp_col]
            data_k = (data_k > 0) * (data_k < 255) * \
                data_k + 255 * (data_k >= 255)
            data_k = np.round(data_k).astype('uint8')
            keys_fim, des_fim = sf.detectAndCompute(data_k, None)
            matches = flann.knnMatch(des_ref, des_fim, k=2)
            scores = np.zeros(len(matches))
            cnt = 0
            for (m0, m1) in matches:
                scores[cnt] = m1.distance / m0.distance
                cnt += 1
            if scores.max() > 2:
                good_points = np.nonzero(scores > 2)[0]
                num_good_points = len(good_points)
                for pnt in good_points:
                    mbest = matches[pnt][0]
                    p_ref = keys_ref[mbest.queryIdx].pt
                    p_fim = keys_fim[mbest.trainIdx].pt
                    refx = p_ref[0] + str_col
                    refy = p_ref[1] + str_row
                    imx = p_fim[0] + str_col
                    imy = p_fim[1] + str_row
                    distance = np.sqrt((refx - imx)**2 + (refy - imy)**2)
                    if (refx >= m) & (refx < m + size_width) & (refy >= k) & (refy < k + size_width) & (
                            imx >= m) & (imx < m + size_width) & (imy >= k) & (imy < k + size_width):
                        maching_pairs.append([[refx, refy], [imx, imy]])
                        distances.append(distance)
    print "There are %d matching pairs." % (len(maching_pairs))

    distances = np.array(distances)
    print distances
    print "Maximum distance: %f, average distance: %f, std: %f." % (distances.max(), distances.mean(), distances.std())
    maching_pairs = np.array(maching_pairs)
    ref_points = maching_pairs[:, 0, :]
    im_points = maching_pairs[:, 1, :]

    return ref_points, im_points, distances


def remapFunction(x_c, y_c, float_points, ref_points):
    dis = (ref_points - float_points)**2
    dis = dis.sum(1)
    dis = np.sqrt(dis)
    dissort = np.sort(dis)
    num_points = dis.size
    idx = np.nonzero((dis > dissort[num_points / 10])
                     * (dis < dissort[-num_points / 10]))[0]
    ref_points = ref_points[idx]
    float_points = float_points[idx]

    num_points = ref_points.shape[0]
    A = np.zeros((num_points, 6))
    x = float_points[:, 0]
    y = float_points[:, 1]
    xout = ref_points[:, 0]
    yout = ref_points[:, 1]

    A[:, 0] = 1.0
    A[:, 1] = x
    A[:, 2] = y
    A[:, 3] = x * y
    A[:, 4] = x**2
    A[:, 5] = y**2

    coefx = np.linalg.lstsq(A, xout)[0]
    coefy = np.linalg.lstsq(A, yout)[0]

    num_points = x_c.size
    A = np.zeros((num_points, 6))
    xms2 = x_c.flatten()
    yms2 = y_c.flatten()
    A[:, 0] = 1.0
    A[:, 1] = xms2
    A[:, 2] = yms2
    A[:, 3] = xms2 * yms2
    A[:, 4] = xms2 ** 2
    A[:, 5] = yms2 ** 2
    xr = np.dot(A, coefx)
    yr = np.dot(A, coefy)
    xr = xr.reshape(x_c.shape)
    yr = yr.reshape(y_c.shape)

    return xr, yr


def findAdjustRemapPanPoints(ref_image, float_image, x_ref, y_ref):

    ref_points, im_points, dis = findKeyPoints(ref_image, float_image)

    x_f, y_f = remapFunction(x_ref, y_ref, ref_points, im_points)
    return x_f, y_f


def callbackL(ros_data):
    global imgL
    width = int(1152 / 2)
    height = int(874 / 2)
    np_arr = np.fromstring(ros_data.data, np.uint8)
    imgL = cv2.resize(cv2.imdecode(
        np_arr, 1), (width, height))


def callbackR(ros_data):
    global imgR
    width = int(1152 / 2)
    height = int(874 / 2)
    np_arr = np.fromstring(ros_data.data, np.uint8)
    imgR = cv2.resize(cv2.imdecode(
        np_arr, 1), (width, height))


def main():
    global imgL, imgR
    while not rospy.is_shutdown():
        if imgL is None or imgR is None:
            print('img is none')
            continue
        lena_org = imgL
        lena_move = imgR
        rw, cl, nb = lena_org.shape
        cv2.imshow("lena", lena_org)
        cv2.imshow("lena move", lena_move)
        over_lay_image = (lena_org.astype('float32') +
                          lena_move.astype('float32')) / 2
        over_lay_image = over_lay_image.astype('uint8')
        cv2.imshow("overlay image", over_lay_image)
        # cv2.waitKey(0)
        b, g, r = cv2.split(lena_org)
        bb, gg, rr = cv2.split(lena_move)
        lena_org_gray = cv2.cvtColor(lena_org, cv2.COLOR_BGR2GRAY)
        lena_move_gray = cv2.cvtColor(lena_move, cv2.COLOR_BGR2GRAY)
        y, x = np.mgrid[:int(874 / 2), :int(1152 / 2)]
        x_m, y_m = findAdjustRemapPanPoints(lena_org_gray, lena_move, x, y)
        # print (x_m, y_m)
        print len(x_m)
        x_m = x_m.astype('float32')
        y_m = y_m.astype('float32')
        lena_register = cv2.remap(lena_move_gray, x_m, y_m, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT,
                                  borderValue=0)
        bb = cv2.remap(bb, x_m, y_m, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT,
                       borderValue=0)
        gg = cv2.remap(gg, x_m, y_m, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT,
                       borderValue=0)
        rr = cv2.remap(rr, x_m, y_m, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT,
                       borderValue=0)
        bgr = cv2.merge((bb, gg, rr))
        over_lay_image = (lena_org_gray.astype('float32') +
                          lena_register.astype('float32')) / 2
        over_lay_image = over_lay_image.astype('uint8')
        images = [bgr, lena_org]
        merge_mertens = cv2.createMergeMertens()
        res_mertens = merge_mertens.process(images)
        cv2.imshow('mertens', res_mertens)
        cv2.imshow("overlay image2", over_lay_image)
        bgr = cv2.cvtColor(over_lay_image, cv2.COLOR_GRAY2BGR)
        cv2.imshow('bgr', bgr)
        k = cv2.waitKey(1) & 0xff
        if k == ord('q'):
            break
        rospy.sleep(0.1)
    cv2.destroyAllWindows()
if __name__ == "__main__":
    rospy.init_node('test_merge')
    # subL = rospy.Subscriber('/bottom/left/image_raw/compressed',
    #                         CompressedImage, callbackL, queue_size=10)
    # subR = rospy.Subscriber('/bottom/right/image_raw/compressed',
    #                         CompressedImage, callbackR, queue_size=10)
    subL = rospy.Subscriber('/stereo/left/image_rect_color/compressed',
                            CompressedImage, callbackL, queue_size=10)
    subR = rospy.Subscriber('/stereo/right/image_rect_color/compressed',
                            CompressedImage, callbackR, queue_size=10)
    main()
