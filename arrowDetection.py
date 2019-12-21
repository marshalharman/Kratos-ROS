import numpy as np
import cv2
import math
black_l = ()
black_u = ()
cap=cv2.VideoCapture(0)
# load image and shrink - it's massive
if cap.isOpened():
	ret,img = cap.read()
else:
	ret = False
while ret:
#    img = cv2.resize(img, None,fx=0.25, fy=0.25, interpolation = cv2.INTER_CUBIC)
#     get a blank canvas for drawing contour on and convert img to grayscale
    canvas = np.zeros(img.shape, np.uint8)
    img2gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #img2gray = cv2.medianBlur(img2gray,5)
    
    ret,thresh = cv2.threshold(cv2.bitwise_not(img2gray),230,255,cv2.THRESH_BINARY)
    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=2)
    cv2.imshow("img2",thresh)
    im2,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    # define main island contour approx. and hull
    #perimeter = cv2.arcLength(contours[0],True)
    #epsilon = 0.01*cv2.arcLength(contours[0],True)
    #approx = cv2.approxPolyDP(contours[0],epsilon,True)

    #hull = cv2.convexHull(contours[0])

    #cv2.isContourConvex(contours[0])
    for c in contours:
        
        cv2.drawContours(canvas, c, -1, (0, 255, 0), 3)
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        
        x, y, w, h = cv2.boundingRect(approx)
        if len(approx)>7 or len(approx)<=6 :
            continue
#        print(len(approx))
        if h >= 15:
            # if height is enough
            # create rectangle for bounding
            rect = (x, y, w, h)
#            rects.append(rect)
            cv2.rectangle(canvas, (x, y), (x+w, y+h), (0, 255, 0), 1);#       cv2.drawContours(canvas, approx, -1, (0, 0, 255), 3)
          ## cv2.drawContours(canvas, hull, -1, (0, 0, 255), 3) # only displays a few points as well.
        arrAngles = []
        arrDist = []
        for i in range(len(approx)):
            x0,y0 = approx[i].ravel()
            x1,y1 = approx[(i+1)%len(approx)].ravel()
            x2,y2 = approx[(i+2)%len(approx)].ravel()
            if (x1-x0)!=0 and (x2-x1)!=0 :
                t1 = (y1-y0)/(x1-x0)
                t2 = (y2-y1)/(x2-x1)
                angle = np.degrees(math.atan(t1-t2/(1+t1*t2)))
            else:
                if (x1-x0)==0 and (x2-x1)!=0:
                    angle2 = np.degrees(math.atan((y2-y1)/(x2-x1)))
                    angle = angle2 - 90
                elif (x1-x0)!=0 and (x2-x1)==0:
                    angle1 = np.degrees(math.atan((y1-y0)/(x1-x0)))
                    angle = 90 - angle1
                else:
                    angle = 0
            arrAngles.append(angle)
            arrDist.append(math.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0)))
#            cv2.circle(canvas,(x0,y0),3,255,-1)
        count = 0
        for i in range(len(arrAngles)):
            if(abs(arrAngles[i])>80 and abs(arrAngles[i])<100):
                count+=1
        if count>=4:
            for i in range(len(approx)):
                x0,y0 = approx[i].ravel()    
                cv2.circle(canvas,(x0,y0),3,255,-1)
#            print(arrAngles,arrDist)
            array_dist_sorted = sorted(arrDist)
            head_length = array_dist_sorted[len(arrDist)-2]
            for i in range(len(arrDist)):
                if head_length == arrDist[i]:
                    x0,y0 = approx[i].ravel()
                    x1,y1 = approx[(i+1)%len(arrDist)].ravel()
            if (x1>x0):
                print("right")
            else :
                print("left")
#            print("SORTED-->")
#            print(array_dist_sorted)
#    corners = cv2.goodFeaturesToTrack(thresh,100,0.01,10)
#    corners = np.int0(corners)
#    print(corners)

    ret,img = cap.read()
    cv2.imshow("feed",img)
    cv2.imshow("Contour", canvas)
    cv2.imshow("Threshold", thresh)
    
    if cv2.waitKey(1) == 27: # exit on ESC
            break
    ret,img = cap.read()

cv2.destroyAllWindows()
cap.release()
