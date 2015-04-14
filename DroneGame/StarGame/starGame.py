# Imports
import sys
import rospy
from PyQt4 import QtCore, QtGui, uic
from threading import Thread
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from random import randint

# Load GUI
formClass = uic.loadUiType("starGameUI.ui")[0]               

# Define use ROS
useROS = True

# Ros spin thread
def rosThread():
	rospy.spin()
	
# My QT class
class AppleGameWindow(QtGui.QMainWindow, formClass):
    def __init__(self, parent=None):
	# Setup QT gui
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)

	# Variables	
	self.gameLength = 60 # seconds
	self.score = 0
	self.highscore = 0
	self.appleRange = 70
	self.minDroneRange = Point(-50,-50,0)
	self.maxDroneRange = Point(780,780,0)
	self.minAppleRange = Point(-50,-50,0)
	self.maxAppleRange = Point(770,770,0)
	centerPoint = (self.maxDroneRange.x-self.minDroneRange.x)/2
	self.dronePos = Point(centerPoint,centerPoint,0)
	self.applePos = Point(0,0,0)	
	self.cropPixels = [369,16, 1020,32, 1000,677, 362,657] # x1,y1,x2,y2,x3,y3,x4,y4 (UL, UR, DR, DL)
	self.bridge = CvBridge()
	self.qpm = QtGui.QPixmap()

	# Setup logo
	logoPixmap = QtGui.QPixmap('images/sdulogo.png')
	logoScaledPixmap = logoPixmap.scaled(logoPixmap.width()/1.7,logoPixmap.height()/1.7, QtCore.Qt.KeepAspectRatio)
	self.labelLogo.setPixmap(logoScaledPixmap)

	# Setup bagground
	self.bgPixmap = QtGui.QPixmap('images/bg.png')
	self.bgPixmap = self.bgPixmap.scaled(900,900, QtCore.Qt.KeepAspectRatio)
	self.labelBackground.setPixmap(self.bgPixmap)

	# Load apple image.
	applePixmap = QtGui.QPixmap('images/apple.png')
	appleScaledPixmap = applePixmap.scaled(100,100, QtCore.Qt.KeepAspectRatio)
	self.labelApple.setPixmap(appleScaledPixmap)
	self.labelApple.setVisible(False)

	# Load drone image.
	dronePixmap = QtGui.QPixmap('images/circle.png') #droneHighResRotated
	droneScaledPixmap = dronePixmap.scaled(100,100, QtCore.Qt.KeepAspectRatio)
	self.labelDrone.setPixmap(droneScaledPixmap)
	self.labelDrone.move(centerPoint,centerPoint)

	# Progress bar
	self.barTime.setTextVisible(False)
	self.barTime.setRange(0,self.gameLength)
	self.updateTime(self.gameLength)

	# Timer stuff	
	self.gameTimer = QtCore.QTimer()
	self.gameTimer.timeout.connect(self.gameTimerCallback)
	self.scoreTimer = QtCore.QTimer()
	self.scoreTimer.timeout.connect(self.scoreTimerCallback)
	self.scoreTimer.start(10)	

	# Bind the event handlers
        self.btnExit.clicked.connect(self.btnExitClicked)  
        self.btnReset.clicked.connect(self.btnResetClicked)  
        self.btnStart.clicked.connect(self.btnStartClicked)  

    def generateApple(self):
	self.applePos.x = randint(self.minAppleRange.x,self.maxAppleRange.x)
	self.applePos.y = randint(self.minAppleRange.y,self.maxAppleRange.y)
	self.labelApple.move(self.applePos.x,self.applePos.y)

    def updateTime(self, time):
	self.barTime.setValue(time)
	self.labelTime.setText(QtCore.QString.number(time))

    def updateScore(self, score):
	self.score = score
	self.lcdScore.display(score)

    def updateHighscore(self):
	if(self.highscore < self.score):
		self.highscore = self.score
		self.lcdHighscore.display(self.highscore)

    def scoreTimerCallback(self):
	if(abs(self.applePos.x-self.dronePos.x)<self.appleRange and abs(self.applePos.y-self.dronePos.y)<self.appleRange and self.labelApple.isVisible() == True):
		self.updateScore(self.score+1)
		self.generateApple()

	self.bgPixmap = self.bgPixmap.scaled(900,900, QtCore.Qt.KeepAspectRatio)
	self.labelBackground.setPixmap(self.bgPixmap)

    def gameTimerCallback(self):
	if(self.barTime.value()-1 < 0):
		self.updateHighscore()
		self.resetGame()
	else:
		self.updateTime(self.barTime.value()-1)

    def keyPressEvent(self, event):
	if(useROS == False):
		# Manual control
		newPos = Point(0,0,0)
		movePixel = 20	
		correctKey = False

		# Check keyboard
		key = event.key()
	    	if key == QtCore.Qt.Key_W:  
			newPos.y = self.dronePos.y - movePixel
			newPos.x = self.dronePos.x 
			correctKey = True
		elif key == QtCore.Qt.Key_D:  
			newPos.x = self.dronePos.x + movePixel
			newPos.y = self.dronePos.y 
			correctKey = True
		elif key == QtCore.Qt.Key_A:  
			newPos.x = self.dronePos.x - movePixel
			newPos.y = self.dronePos.y 
			correctKey = True
		elif key == QtCore.Qt.Key_S:  
			newPos.y = self.dronePos.y + movePixel
			newPos.x = self.dronePos.x 
			correctKey = True

		# Check if move is allowed (inside the flying area)
		if(correctKey == True):
			if(self.minDroneRange.x < newPos.x and newPos.x < self.maxDroneRange.x and self.minDroneRange.y < newPos.y and newPos.y < self.maxDroneRange.y):
				self.moveDrone(newPos)

	key = event.key()
    	if key == QtCore.Qt.Key_Escape:
		self.btnExitClicked()  
	elif key == QtCore.Qt.Key_R:
		self.btnResetClicked()  	
	elif key == QtCore.Qt.Key_N:
		self.btnStartClicked()  

    def btnExitClicked(self):    
	rospy.signal_shutdown("Exit program!")
	QtGui.QApplication.quit()

    def btnResetClicked(self):  
	self.resetGame()

    def resetGame(self):
	# Reset score and time
	#self.updateScore(0)
	self.updateTime(self.gameLength)

	# Stop timers
	self.gameTimer.stop()

	# Hide the apple
	self.labelApple.setVisible(False)

    def btnStartClicked(self):  
	# Reset score and time
	self.updateScore(0)
	self.updateTime(self.gameLength)

	# Generate first apple and set visibility
	self.generateApple()
	self.labelApple.setVisible(True)

	# Start game timers
	self.gameTimer.start(1000)	# Timeout every 1000m

    def moveDrone(self, msg):
	if useROS == True:
		self.labelDrone.move(msg.y*3-70, 900-msg.x*3-70)
		self.dronePos.x = msg.y*3-70
		self.dronePos.y = 900-msg.x*3-70
	else:
		self.labelDrone.move(msg.x, msg.y)
		self.dronePos.x = msg.x
		self.dronePos.y = msg.y

    def pointCallback(self, msg):
	self.moveDrone(msg) 

    def imageCallback(self, image):
	#self.cropPixels = [369,16, 1020,32, 1000,677, 362,657] # x1,y1,x2,y2,x3,y3,x4,y4 (UL, UR, DR, DL)

	# Convert to opencv and crop image
	cvImage = self.bridge.imgmsg_to_cv2(image)
	#cvImage = cvImage[self.cropPixels[1]:self.cropPixels[5], self.cropPixels[0]:self.cropPixels[4]]
	cvImage = cvImage[5:670+5,360:676+360]
	# [startY:endY, startX:endX] 

	# Rotate image	
	rows,cols,channels = cvImage.shape
	M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
	cvImage = cv2.warpAffine(cvImage,M,(cols,rows))

	# Convert to QImage and then QPixmap
	cvImage = cv2.cvtColor(cvImage, cv2.cv.CV_BGR2RGB)
	qimg = QtGui.QImage(cvImage.data, cvImage.shape[1], cvImage.shape[0], QtGui.QImage.Format_RGB888)
	self.bgPixmap = QtGui.QPixmap.fromImage(qimg)

# Main
if __name__ == "__main__":
	# Load QT-Gui and game window
	app = QtGui.QApplication(sys.argv)
	gameWindow = AppleGameWindow(None)

	if useROS == True:
		# Init ROS
		rospy.init_node('appleGameNode', anonymous=True)
		rospy.Subscriber("positionPublisher5", Point, gameWindow.pointCallback)
		rospy.Subscriber("imagePublisher", Image, gameWindow.imageCallback)

		# Start ros_spin in new thread since it is a blocking call
		threadROS = Thread(target = rosThread)
		threadROS.start()

	# Show game window
	gameWindow.show()
	gameWindow.showFullScreen()
	app.exec_()
