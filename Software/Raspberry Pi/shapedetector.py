import cv2

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self,c):
        #initialize the shape name and approximate contour
        shape ="undefined"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.001 * peri, True) #0.04

        if len(approx) < 50:
            shape = "triangle"
        elif len(approx) < 80:
            (x,y,w,h) = cv2.boundingRect(approx)
            ar = w / float(h)
            shape = "square" if ar >= 0.95 and ar <=1.05 else "rectangle"
        elif len(approx) < 100:
            shape = "pentagon"
        elif len(approx >= 100):
            shape = "circle"

        return shape
