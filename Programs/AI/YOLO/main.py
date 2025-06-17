import numpy as np
from ultralytics import YOLO
from functools import cmp_to_key
import cv2

# NN
name = "human_ncnn_model"
model = YOLO(name)

#color & detection settings
fon_hsv_min = np.array([43, 51, 102], np.uint8) # blue
fon_hsv_max = np.array([153, 250, 250], np.uint8)

uzor_hsv_min = np.array([38, 104, 153], np.uint8) #fiolet
uzor_hsv_max = np.array([200, 153, 200], np.uint8)
minUzorArea = 1000

debugFindObj = 1

defineRightLim = 450
defineLeftLim = 180


#robot parameters
r = 0.158 #wheel radius
lx =  0.787 # length of robot axis

x = 0
xN = 0
teta = 0
tetaN = 0

cF = 50
cR = 0.1*2*3.14

safatyDist = 20

def bbArea(bb1, bb2):
    if ((bb1[2]*bb1[3]) > (bb2[2]*bb2[3])):
        return 1
    elif ((bb1[2]*bb1[3]) < (bb2[2]*bb2[3])):
        return -1
    else:
        return 0


def findObject(img: np.array):
    isFind = False
    Cx = 0
    Cy = 0

    # nn part

    results = model.predict(img)

    boundingBoxes = [result.boxes.xywh.numpy()  for result in results]
    boundingBoxes.sort(key=cmp_to_key(bbArea), reverse=True)

    x, y, w, h = boundingBoxes[0]

    y1Point = y - (h/2)
    y2Point = y + (h/2)
    x1Point = x - (w/2)
    x2Point = x + (w/2)

    humanImg = img[y1Point:y2Point, x1Point:x2Point]

    # color detection part
    hsv = cv2.cvtColor(humanImg, cv2.COLOR_BGR2HSV)

    thresh = cv2.inRange(hsv, fon_hsv_min, fon_hsv_max)

    contours, hierarchy = cv2.findContours( thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    #print(cv2.contourArea(contours[0]))
    x, y, w, h = cv2.boundingRect(contours[0])


    thresh2 = cv2.inRange(hsv, uzor_hsv_min, uzor_hsv_max)
    uzor_img = thresh2[y:y + h, x:x + w]

    contoursUzor, hierarchyUzor = cv2.findContours(uzor_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contoursUzor = sorted(contoursUzor, key=cv2.contourArea, reverse=True)

    if ((len(contoursUzor) > 0) and (cv2.contourArea(contoursUzor[0]) > minUzorArea)):

        if debugFindObj:
            cv2.imshow("uzor", uzor_img)
            testedImg = img[y:y + h, x:x + w]
            cv2.imshow("testedImg", testedImg)
            # отображаем контуры поверх изображения
            cv2.drawContours(testedImg, contoursUzor, -1, (0, 127, 0), 3, cv2.LINE_AA, hierarchyUzor, 1)
            cv2.imshow('contours', testedImg)  # выводим итоговое изображение в окно

        isFind = True
        Cx = (x+w)/2
        Cy = (y+h)/2
    return (isFind, Cx, Cy)


def getDistanceToPoint(x, y):
    return safatyDist

def calkVelocities(x, xn, teta, tetan, r, lx, dt=1000):

    v = 0.01*(xn-x)/dt
    omega = (tetan - teta)/dt

    wl = (1/r)*(v - (omega*lx)/2)
    wr = (1/r)*(v + (omega*lx)/2)

    return wl, wr

def sendToLocalPID(wl, wr):
    pass

def sound():
    pass

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while True:
        flag, img = cap.read() #cv2.imread("testImg.bmp")
        isFind, Cx, Cy = findObject(img.copy())

        if debugFindObj:
            cv2.line(img, (defineLeftLim, 0), (defineLeftLim, 480), (255,255,0))
            cv2.line(img, (defineRightLim, 0), (defineRightLim, 480), (255, 255, 0))
            cv2.circle(img, (int(Cx), int(Cy)), 5, (0, 0, 255), -1)
            cv2.imshow("camera", img)

        if isFind:
            d = getDistanceToPoint(Cx, Cy)
            if (d <= 20):

                if ((Cx >= defineLeftLim) and (Cx <= defineRightLim)):
                    xN = x + cF
                    tetaN = teta
                elif (Cx < defineLeftLim):
                    xN = x
                    tetaN = teta + cR

                elif (Cx > defineRightLim):
                    xN = x
                    tetaN = teta - cR

                wl, wr = calkVelocities(x, xN, teta, tetaN, r, lx)
                sendToLocalPID(wl, wr)
                x = xN
                teta = tetaN
            elif (d > 20):
                sound()

       # print(ret)
        if cv2.waitKey(1) == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()