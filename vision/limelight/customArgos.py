import cv2
import numpy as np

# global variables go here:
testVar = 0
H_min = 45
H_max = 75
S_min = 186
S_max = 255
V_min = 71
V_max = 242

cameraMatrix = np.array([[772.53876202/3.0, 0, 479.132337442/3.0], [0, 769.052151477/3.0, 359.143001808/3.0], [0, 0, 1.0]])
distortionCoeff = np.array([[2.9684613693070039e-01, -1.4380252254747885e+00,-2.2098421479494509e-03,
                        -3.3894563533907176e-03, 2.5344430354806740e+00]])

distortedPts1 = np.float32([[-30, -30], [160, 0], [0, 240], [160, 240]])
desiredPts1 = np.float32([[0, 0], [160, 0], [0, 240], [160, 240]])
distortedPts2 = np.float32([[160, 0], [350, 30], [320, 240], [160, 240]])
desiredPts2 = np.float32([[160, 0], [320, 0], [320, 240], [160, 240]])


def getTargetAtTop(box):
    blx = box[0][0]
    bly = box[0][1]
    brx = box[1][0]
    bry = box[1][1]
    
    px = (blx + brx)/2.0
    py = (bly + bry)/2.0
    nx = (1/160) * (px - 159.5)
    ny = (1/120) * (119.5 - py)

    vpw = 2.0*np.tan(H_FOV/2)
    vph = 2.0*np.tan(V_FOV/2)

    nx = vpw/2 * nx;
    ny = vph/2 * ny;

    tx = np.atan2(1, nx)
    ty = np.atan2(1, ny)
    
    return px, py, tx, ty
# To change a global variable inside a function,
# re-declare it the global keyword
def debugData(box):
    global testVar
    testVar = testVar + 1
    if testVar == 100:
        print("test")
        if box.size:
            print(box[0])
    if testVar >= 200:
        print("print")
        testVar = 0

def drawDecorations(image):
    cv2.putText(image,
        'ArgosVision!',
        (210, 20),
        cv2.FONT_HERSHEY_TRIPLEX,
        .5, (0, 255, 0), 1, cv2.LINE_AA)

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):

    dst = image
    
    #Apply Perspective Transforms
    matrix1 = cv2.getPerspectiveTransform(distortedPts1, desiredPts1)
    matrix2 = cv2.getPerspectiveTransform(distortedPts1, desiredPts1)
    matrix = matrix1 * matrix2
    undistorted = cv2.warpPerspective(dst, matrix, (320, 240))

    #hd,  wd = image.shape[:2]
    #newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distortionCoeff, (wd,hd), 1, (wd,hd))

    # undistort
    #dst = cv2.undistort(image, cameraMatrix, distortionCoeff, None, newcameramtx)

    # method-2
    #mapx,mapy=cv2.initUndistortRectifyMap(cameraMatrix,distortionCoeff,None,newcameramtx,(wd,hd),5)
    #dst = cv2.remap(image,mapx,mapy,cv2.INTER_LINEAR)


    img_hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (H_min, S_min, V_min), (H_max, S_max, V_max))

    kernel = np.ones((1, 1), np.uint8)
    #img_threshold = cv2.morphologyEx(img_threshold, cv2.MORPH_OPEN, kernel)
    img_threshold = cv2.dilate(img_threshold, kernel, iterations=1)

    contours, _ = cv2.findContours(img_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    groupedSmartContour = np.array([[]])
    fittedRedBox = np.array([[]])

    llpython = [0,0,0,0,0,0,0,0]
    px, py, tx, ty, isValid = 0, 0, 0, 0, 0

    if (len(contours) > 0 and len(contours) < 7):
        filteredCountours = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h

            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            totalArea = w * h
            areaPct = float(area) / totalArea

            if aspect_ratio > 0.3 and aspect_ratio < 5.0:
                if areaPct > 0.001 and areaPct < 10.0:
                    filteredCountours.append(cnt)

        groupedSmartContour = np.concatenate(filteredCountours)
        #cv2.drawContours(image, groupedSmartContour, -1, (0,255,0), 2)
        #largestContour = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(groupedSmartContour)

        cv2.rectangle(dst,(x,y),(x+w,y+h),(0,255,255),2)
        rect = cv2.minAreaRect(groupedSmartContour)
        fittedRedBox = cv2.boxPoints(rect)
        fittedRedBox = np.int0(fittedRedBox)
        cv2.drawContours(dst,[fittedRedBox],0,(0,0,255),2)
        
        #send the position of the top of the hub along with center
        if fittedRedBox.size:
          isValid = 1
          px, py, tx, ty = getTargetAtTop(fittedRedBox)
        llpython = [isValid,px,py,tx,ty,9,8,7]

    debugData(fittedRedBox)
    drawDecorations(dst)

    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    return groupedSmartContour, undistorted, llpython
