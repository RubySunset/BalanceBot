import numpy as np
import cv2
import matplotlib.pyplot as plt

class VisionAlgorithm():
	def __init__(self, image):
		self.image = cv2.imread(image)
		self.image = cv2.resize(self.image, (0, 0), fx=0.5, fy=0.5)

	def coordinateCalculate(self, slope, intercept):
		y1 = self.image.shape[0]
		y2 = int(y1 * (3.4/5))
		x1 = int((y1-intercept) / slope)
		x2 = int((y2-intercept) / slope)
		return np.array([x1, y1, x2, y2])

	def laneDetection(self):
		gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
		edges = cv2.Canny(gray, 50, 150)
		lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=45)
		for line in lines:
			x1, y1, x2, y2 = line[0]
			cv2.line(self.image, (x1, y1), (x2, y2), (0,255, 0), 3)		
	
	def showImage(self):
			cv2.imshow("A", self.image)
			cv2.waitKey(0)
			cv2.destroyAllWindows()

	def colourDetection(self, colour):
		colour_ranges = {
    				"yellow": (np.array([18, 50, 50]), np.array([48, 255, 255])),
    				"red": (np.array([0, 50, 50]), np.array([10, 255, 255])),
    				"blue": (np.array([90, 50, 50]), np.array([100, 255, 255]))
			       	}
		hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
		(lower, upper) = colour_ranges[colour]
		mask = cv2.inRange(hsv, lower, upper)
		self.image = cv2.bitwise_and(self.image, self.image, mask=mask)
		
