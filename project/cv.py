import os
import cv2
path="br/"
files=os.listdir(path)
x=[]
y=[]
for i in files:
    if i.endswith(".jpg"):
        temp=cv2.imread(path+i,0)
        x.append(temp)
        y.append(i[:-4])

print(len(x))
print(len(y))

# creating dataset


from TOOLS import Functions
import numpy as np
import math
import argparse

for q in range(len(x)):
    img=imgk=x[q]
    value=img
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    topHat = cv2.morphologyEx(value, cv2.MORPH_TOPHAT, kernel)
    blackHat = cv2.morphologyEx(value, cv2.MORPH_BLACKHAT, kernel)
    add = cv2.add(value, topHat)
    subtract = cv2.subtract(add, blackHat)
    blur = cv2.GaussianBlur(subtract, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 19, 9)
    a,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # get height and width
    height, width = thresh.shape

    # create a numpy array with shape given by threshed image value dimensions
    imageContours = np.zeros((height, width, 3), dtype=np.uint8)

    # list and counter of possible chars
    possibleChars = []
    countOfPossibleChars = 0

    # loop to check if any (possible) char is found
    for i in range(0, len(contours)):

        # draw contours based on actual found contours of thresh image
        cv2.drawContours(imageContours, contours, i, (255, 255, 255))

        # retrieve a possible char by the result ifChar class give us
        possibleChar = Functions.ifChar(contours[i])

        # by computing some values (area, width, height, aspect ratio) possibleChars list is being populated
        if Functions.checkIfChar(possibleChar) is True:
            countOfPossibleChars = countOfPossibleChars + 1
            possibleChars.append(possibleChar)

    # cv2.imshow("contours", imageContours)
    # cv2.imwrite(temp_folder + '8 - imageContours.png', imageContours)

    imageContours = np.zeros((height, width, 3), np.uint8)

    ctrs = []

    # populating ctrs list with each char of possibleChars
    for char in possibleChars:
        ctrs.append(char.contour)

    # using values from ctrs to draw new contours
    cv2.drawContours(imageContours, ctrs, -1, (255, 255, 255))
    # cv2.imshow("contoursPossibleChars", imageContours)
    # cv2.imwrite(temp_folder + '9 - contoursPossibleChars.png', imageContours)

    plates_list = []
    listOfListsOfMatchingChars = []

    for possibleC in possibleChars:
        def matchingChars(possibleC, possibleChars):
            listOfMatchingChars = []
            for possibleMatchingChar in possibleChars:
                if possibleMatchingChar == possibleC:
                    continue
                distanceBetweenChars = Functions.distanceBetweenChars(possibleC, possibleMatchingChar)
                angleBetweenChars = Functions.angleBetweenChars(possibleC, possibleMatchingChar)
                changeInArea = float(abs(possibleMatchingChar.boundingRectArea - possibleC.boundingRectArea)) / float(
                    possibleC.boundingRectArea)
                changeInWidth = float(abs(possibleMatchingChar.boundingRectWidth - possibleC.boundingRectWidth)) / float(
                    possibleC.boundingRectWidth)
                changeInHeight = float(abs(possibleMatchingChar.boundingRectHeight - possibleC.boundingRectHeight)) / float(
                    possibleC.boundingRectHeight)
                if distanceBetweenChars < (possibleC.diagonalSize * 5) and \
                        angleBetweenChars < 12.0 and \
                        changeInArea < 0.5 and \
                        changeInWidth < 0.8 and \
                        changeInHeight < 0.2:
                    listOfMatchingChars.append(possibleMatchingChar)
            return listOfMatchingChars
        listOfMatchingChars = matchingChars(possibleC, possibleChars)
        listOfMatchingChars.append(possibleC)
        if len(listOfMatchingChars)!=7:#<5 and len(listOfMatchingChars)>10:
            continue
        listOfListsOfMatchingChars.append(listOfMatchingChars)
    imageContours = np.zeros((height, width, 3), np.uint8)

    for listOfMatchingChars in listOfListsOfMatchingChars:
        contoursColor = (255, 0, 255)

        contours = []

        for matchingChar in listOfMatchingChars:
            contours.append(matchingChar.contour)

        cv2.drawContours(imageContours, contours, -1, contoursColor)

    # cv2.imshow("finalContours", imageContours)
    # cv2.imwrite(temp_folder + '10 - finalContours.png', imageContours)
    h = 0

    for listOfMatchingChars in listOfListsOfMatchingChars:
        possiblePlate = Functions.PossiblePlate()

        # sort chars from left to right based on x position
        listOfMatchingChars.sort(key=lambda matchingChar: matchingChar.centerX)

        # calculate the center point of the plate
        plateCenterX = (listOfMatchingChars[0].centerX + listOfMatchingChars[len(listOfMatchingChars) - 1].centerX) / 2.0
        plateCenterY = (listOfMatchingChars[0].centerY + listOfMatchingChars[len(listOfMatchingChars) - 1].centerY) / 2.0

        plateCenter = plateCenterX, plateCenterY

        # calculate plate width and height
        plateWidth = int((listOfMatchingChars[len(listOfMatchingChars) - 1].boundingRectX + listOfMatchingChars[
            len(listOfMatchingChars) - 1].boundingRectWidth - listOfMatchingChars[0].boundingRectX) * 1.3)

        totalOfCharHeights = 0

        for matchingChar in listOfMatchingChars:
            totalOfCharHeights = totalOfCharHeights + matchingChar.boundingRectHeight

        averageCharHeight = totalOfCharHeights / len(listOfMatchingChars)

        plateHeight = int(averageCharHeight * 1.5)

        # calculate correction angle of plate region
        opposite = listOfMatchingChars[len(listOfMatchingChars) - 1].centerY - listOfMatchingChars[0].centerY

        hypotenuse = Functions.distanceBetweenChars(listOfMatchingChars[0],
                                                    listOfMatchingChars[len(listOfMatchingChars) - 1])
        if hypotenuse != 0:
            correctionAngleInRad = math.asin(opposite / hypotenuse)
        else:
            correctionAngleInRad=0
        correctionAngleInDeg = correctionAngleInRad * (180.0 / math.pi)

        # pack plate region center point, width and height, and correction angle into rotated rect member variable of plate
        possiblePlate.rrLocationOfPlateInScene = (tuple(plateCenter), (plateWidth, plateHeight), correctionAngleInDeg)

        # get the rotation matrix for our calculated correction angle
        rotationMatrix = cv2.getRotationMatrix2D(tuple(plateCenter), correctionAngleInDeg, 1.0)

        height, width = img.shape

        # rotate the entire image
        imgRotated = cv2.warpAffine(imgk, rotationMatrix, (width, height))

        # crop the image/plate detected
        imgCropped = cv2.getRectSubPix(imgRotated, (plateWidth, plateHeight), tuple(plateCenter))

        # copy the cropped plate image into the applicable member variable of the possible plate
        possiblePlate.Plate = imgCropped
        cv2.imwrite("a/"+y[q]+".jpg",imgCropped)
        break
        # populate plates_list with the detected plate
        if possiblePlate.Plate is not None:
            plates_list.append(possiblePlate)

        # draw a ROI on the original image
        for i in range(0, len(plates_list)):
            # finds the four vertices of a rotated rect - it is useful to draw the rectangle.
            p2fRectPoints = cv2.boxPoints(plates_list[i].rrLocationOfPlateInScene)

            # roi rectangle colour
            rectColour = (0, 255, 0)

            cv2.line(imageContours, tuple(p2fRectPoints[0]), tuple(p2fRectPoints[1]), rectColour, 2)
            cv2.line(imageContours, tuple(p2fRectPoints[1]), tuple(p2fRectPoints[2]), rectColour, 2)
            cv2.line(imageContours, tuple(p2fRectPoints[2]), tuple(p2fRectPoints[3]), rectColour, 2)
            cv2.line(imageContours, tuple(p2fRectPoints[3]), tuple(p2fRectPoints[0]), rectColour, 2)

            cv2.line(img, tuple(p2fRectPoints[0]), tuple(p2fRectPoints[1]), rectColour, 2)
            cv2.line(img, tuple(p2fRectPoints[1]), tuple(p2fRectPoints[2]), rectColour, 2)
            cv2.line(img, tuple(p2fRectPoints[2]), tuple(p2fRectPoints[3]), rectColour, 2)
            cv2.line(img, tuple(p2fRectPoints[3]), tuple(p2fRectPoints[0]), rectColour, 2)

            # cv2.imshow("detected", imageContours)
            # cv2.imwrite(temp_folder + '11 - detected.png', imageContours)
            # cv2.imwrite(temp_folder + '12 - detectedOriginal.png', img)

            # cv2.imshow("plate", plates_list[i].Plate)
            # cv2.imwrite(temp_folder + '13 - plate.png', plates_list[i].Plate)
