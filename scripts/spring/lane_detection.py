from numpy import ndarray
import cv2 as cv
import numpy as np
from utils import cols, rows
from spring_line_pkg.cfg import BlobConfig

def compute_lines(self, config:BlobConfig, image:ndarray) -> ndarray:
        lines_mat = np.zeros_like(image)
        x = 0
        y = int(config.lines_top * rows(image))
        w = cols(image)
        h = int(rows(image) - config.lines_top * rows(image))

        # gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        canny_image = apply_filters(image,config)
        canny_image = get_region_of_interest(canny_image)
        cv.imshow("Canny", canny_image)

        image_cropped = canny_image[y:y+h, x:x+w]

        lines = cv.HoughLinesP(image_cropped,
                               rho=config.lines_rho,
                               theta=0.01745329251,
                               threshold=config.lines_thresh,
                               minLineLength=config.lines_min_len,
                               maxLineGap=config.lines_max_gap)
        if lines is not None:
            for l in lines:
                l = l[0] # (4,1) => (4,)
                diffx = l[0] - l[2]
                diffy = l[1] - l[3]

                slope = diffy / diffx

                if abs(slope) < config.lines_min_slope: continue

                diffx *= 5
                diffy *= 5

                l[0] -= diffx
                l[1] -= diffy
                l[2] += diffx
                l[3] += diffy

                cv.line(
                    lines_mat,
                    (l[0], int(l[1] + config.lines_top * rows(image))),
                    (l[2], int(l[3] + config.lines_top * rows(image))),
                    255, 5)

        return lines_mat


def apply_white_balance(cv_image):

    # convert image to the LAB color space
    lab_image = cv.cvtColor(cv_image, cv.COLOR_BGR2LAB)

    average_a = np.average(lab_image[:,:,1])
    average_b = np.average(lab_image[:,:,2])

    lab_image[:,:,1] = lab_image[:,:,1] - ((average_a - 128) * (lab_image[:,:,0] / 255.0) * 1.1)
    lab_image[:,:,2] = lab_image[:,:,2] - ((average_b - 128) * (lab_image[:,:,0] / 255.0) * 1.1)

    return cv.cvtColor(lab_image, cv.COLOR_LAB2BGR)

def apply_filters(cv_image, RC:BlobConfig):

    # helps remove some of the yellow from the sunlight
    balanced_image = apply_white_balance(cv_image)

    # one more time
    # balanced_image = apply_white_balance(balanced_image)

    # convert image to the HLS color space
    hls_image = cv.cvtColor(balanced_image, cv.COLOR_BGR2HLS)

    # lower and upper bounds for the color white
    lower_bounds = np.uint8([0, RC.light_low, 0])
    upper_bounds = np.uint8([255, 255, 255])
    white_detection_mask = cv.inRange(hls_image, lower_bounds, upper_bounds)

    # lower and upper bounds for the color yellow
    # lower_bounds = np.uint8([10, 0, 100])
    # upper_bounds = np.uint8([40, 255, 255])
    # yellow_detection_mask = cv.inRange(hls_image, lower_bounds, upper_bounds)

    # combine the masks
    # white_or_yellow_mask = cv.bitwise_or(white_detection_mask, yellow_mask)
    balanced_image_with_mask =  cv.bitwise_and(balanced_image, balanced_image, mask = white_detection_mask)

    # convert image to grayscale
    gray_balanced_image_with_mask = cv.cvtColor(balanced_image_with_mask, cv.COLOR_BGR2GRAY)

    # smooth out the image
    kernel = np.ones((5, 5), np.float32) / 25
    smoothed_gray_image = cv.filter2D(gray_balanced_image_with_mask, -1, kernel)

    # find and return the edges in in smoothed image
    return cv.Canny(smoothed_gray_image, 200, 255)

def get_region_of_interest(image):

    width = image.shape[1]
    height = image.shape[0]

    width = width / 8
    height = height / 8

    roi = np.array([[

                       [0, height*8],
                       [0, height*5],
                       [width*2, (height*4)-30],
                       [width*5 , (height*4)-30],
                       [width*8, height*7],
                       [width*8, height*8]

                   ]], dtype = np.int32)

    mask = np.zeros_like(image)
    cv.fillPoly(mask, roi, 255)

    # return the image with the region of interest
    return cv.bitwise_and(image, mask)



# def find_lanes(input_image: ndarray,
#                config: BlobConfig,
#                debug_image: ndarray=None) -> ndarray:
#     """
#     This algorithm uses light-on-dark contrast to find
#     lane lines. If lanes do not have this property, another
#     lane-finding algorithm may be used instead
#     """

#     # Median blur
#     #image = cv.medianBlur(input_image, config.enhance_blur * 2 + 1)


#     ## Find edges using Laplacian and Sobel
#     # Canny could also be used here but the Laplacian/Sobel
#     # approach typically yeilds improved expiremental
#     # results for this case
#     # image = cv.Laplacian(image, -1, config.lapla_ksize * 2 + 1)
#     # image = cv.Sobel(image, -1, config.sobel_xorder,
#     #             config.sobel_yorder,
#     #             config.sobel_ksize * 2 + 1)
#     gray_image = cv.cvtColor(input_image, cv.COLOR_BGR2GRAY)
#     image = cv.Canny(gray_image, 200,255)

#     # Dilate images

#     # dilation_size = (2 * config.blob_dilation_size + 1, 2 * config.blob_dilation_size + 1)
#     # dilation_anchor = (config.blob_dilation_size, config.blob_dilation_size)
#     # dilate_element = cv.getStructuringElement(cv.MORPH_RECT, dilation_size, dilation_anchor)
#     # image = cv.dilate(image, dilate_element)
#     image = compute_lines(image, config, debug_image=debug_image)

#     return image
