import numpy as np
import cv2

vs = cv2.VideoCapture(0)

first_frame = None
kernel = np.ones((5, 5))

while vs.isOpened():

    _, frame_1 = vs.read()
    _, frame_2 = vs.read()
    diff_frame = cv2.absdiff(frame_1, frame_2)
    diff_gray = cv2.cvtColor(diff_frame, cv2.COLOR_BGR2GRAY)
    diff_blur = cv2.GaussianBlur(diff_gray, (5, 5), 0)
    diff_dilated = cv2.dilate(diff_blur, kernel, 1)
    thresh_frame = cv2.threshold(diff_dilated, thresh=20, maxval=255, type=cv2.THRESH_BINARY)[1]

    contours, hierarchy = cv2.findContours(thresh_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        print('Movement detected')
    else:
        print('No movement')

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if cv2.contourArea(contour) > 300:
            print(contour)
            cv2.rectangle(frame_1, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.imshow('Detection', frame_1)
    cv2.imshow('Difference', diff_frame)
    cv2.imshow('Threshold', thresh_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        vs.release()
        cv2.destroyAllWindows()
