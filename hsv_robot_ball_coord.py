
import serial
import numpy as np
import cv2
import math
import time
ser = serial.Serial("COM8", 9600)
xp = 100
yp = 100

cv2.namedWindow("mask")

def nothing(x):
    pass
# [2,28,109] [26,90, 252]



# low_hsv =  (0, 173, 2)
# high_hsv =  (50, 255, 250)
low_hsv =  (4, 44, 5)
high_hsv =  (18, 247, 255)




cam = cv2.VideoCapture("http://192.168.4.1:81")
# cam = cv2.VideoCapture(0)
success, frame = cam.read()
#print(frame.shape)

calibration_distance = 67 # см
calibration_linear_size = 81 # pixel

def img_to_local_coord(x_px, l): # [l] = см
    Wpx = 640
    Hpx = 480
    H = 21.8 # высота камеры в см
    cam_angle_x = 90 # в градусах, угол бетта. При подстановке а тангенс перевести в радианы
    tan_cam_half_angle_x = 0.585
    try:
        y = math.sqrt(abs(l ** 2 - H ** 2))
    except Exception as e:
        y = 0
        print(e)


    print(y)

    x = y * 2 * (x_px - Wpx / 2) * tan_cam_half_angle_x / Wpx # math.tan(math.radians(cam_angle_x) / 2)
    return [round(x), round(y)] # в сантиметрах

t = time.monotonic()

while 1:

    if success == False:
        print(frame)
        continue
    # frame = cv2.imread('ball.png')
    #frame[100 : 550, 100 : 550, 0] = 240
    #frame[:, :, 2] += 50

    #print(frame.shape)

    #frame = 255 - frame

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, low_hsv, high_hsv)
    cv2.imshow("mask", mask)

    connectivity = 4
    # Perform the operation
    output = cv2.connectedComponentsWithStats(mask, connectivity, cv2.CV_32S)

    # Get the results
    # The first cell is the number of labels
    num_labels = output[0]
    # The second cell is the label matrix
    labels = output[1]
    # The third cell is the stat matrix
    stats = output[2]

    filtered = np.zeros_like(mask)
    g = 0
    for i in range(1, num_labels):
        a = stats[i, cv2.CC_STAT_AREA]
        t = stats[i, cv2.CC_STAT_TOP]
        l = stats[i, cv2.CC_STAT_LEFT]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        #print(a)

        if (a >= 600):
            g = 1
            filtered[np.where(labels == i)] = 255
            #print(a)
            linear_size = max(w, h)

            distance_by_cam = round(calibration_distance * calibration_linear_size / linear_size)
            cv2.putText(frame, f"dist={distance_by_cam}", (l + w + 10, t + h + 10), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"lsize={int(linear_size)}", (l - 60, t + h + 10), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 255), 2, cv2.LINE_AA)

            cv2.putText(frame, str(a), (l, t), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.rectangle(frame, (l, t), (l + w, t + h), (0, 255, 0), 2)
            x_b, y_b = img_to_local_coord(l + w / 2, distance_by_cam)
            cv2.putText(frame, f"{x_b}, {y_b}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
    #ser.write(f"{1} {1}".encode())
    x, y = x_b, y_b
    print("    ", g)

    if not g:
        print(1)
        print(time.monotonic() - t >= 0.4, "fhdsihuhfjkskdj")
        if time.monotonic() - t >= 0.4:
            ser.write(f"{30}       {-30}".encode())
            t = time.monotonic()
    else:
        ser.write(f"{1}       {1}".encode())

    #ser.write(f"{1} {1}".encode())
    xp, yp = x, y
    success, frame = cam.read()

    cv2.imshow("frame", frame)
    #cv2.imshow("hsv", hsv[:, :, 0])
    cv2.imshow("filtered", filtered)

    key = cv2.waitKey(10)

    if (key == ord(' ')):
        break

cam.release()
cv2.destroyAllWindows()
cv2.waitKey(10)
