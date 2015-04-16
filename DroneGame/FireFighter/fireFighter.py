# Imports
import sys
import rospy
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import QObject, pyqtSignal
from threading import Thread
from geometry_msgs.msg import Point
from random import randint
from math import sqrt

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
# Load GUI
formClass = uic.loadUiType("fireFighterUI.ui")[0]               

# Ros spin thread
def rosThread():
	rospy.spin()
	
class AnimalObj:
	def __init__(self,farmobj,  obj, combinedImg):
		self.farmobj = farmobj
		self.obj = obj
		self.combinedImg = combinedImg
		self.rescued = False

	def setRescued(self, rescued):
		self.rescued = rescued

	def isRescued(self):
		return self.rescued

	def hideFarm(self):
		self.farmobj.hide()

	def showFarm(self):
		self.farmobj.show()
# My QT class
class StarGameWindow(QtGui.QMainWindow, formClass):
    trigger = pyqtSignal(str)
    triggerFarm = pyqtSignal(object)
    triggerBackgroundUpdater = pyqtSignal()

    def __init__(self, parent=None):
	# Setup QT gui
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)

	self.CarryingAnimal = None

	self.bridge = CvBridge()

	self.triggerFarm.connect(self.handle_trigger_farm)
	self.triggerBackgroundUpdater.connect(self.handle_background_update)

	self.labelBackground.lower()

	# Variables	
	self.gameRunning = False
	self.score = 0
	self.highscore = 0
	self.xOffset = 800
	self.yOffset = 100
	self.animalLabels = []

	# Create list of animalObjects
	self.animalLabels.append( AnimalObj( self.labelAnimalCowFarm, self.labelAnimalCow, "images/droneAnimalCowCombined.png") )
	self.animalLabels.append( AnimalObj( self.labelAnimalTigerFarm, self.labelAnimalTiger, "images/droneAnimalTigerCombined.png") )
	self.animalLabels.append( AnimalObj( self.labelAnimalRabbidFarm, self.labelAnimalRabbid, "images/droneAnimalRabbidCombined.png") )


	# Setup logo
	logoPixmap = QtGui.QPixmap('images/sdulogo.png')
	logoScaledPixmap = logoPixmap.scaled(logoPixmap.width()/2,logoPixmap.height()/2, QtCore.Qt.KeepAspectRatio)
	self.labelLogo.setPixmap(logoScaledPixmap)

	# Get screen info (resolution)
	self.screen = QtGui.QDesktopWidget().screenGeometry()

	# Load drone image.
	dronePixmap = QtGui.QPixmap('images/droneHighRes.png')
	self.labelDrone.setPixmap(dronePixmap)




	tmpPixmap = QtGui.QPixmap('images/animalRabbid.png')
	self.labelAnimalRabbid.setPixmap(tmpPixmap)

	tmpPixmap = QtGui.QPixmap('images/animalCow.png')
	self.labelAnimalCow.setPixmap(tmpPixmap)

	tmpPixmap = QtGui.QPixmap('images/animalTiger.png')
	self.labelAnimalTiger.setPixmap(tmpPixmap)



	tmpPixmap = QtGui.QPixmap('images/animalRabbid.png')
	self.labelAnimalRabbidFarm.setPixmap(tmpPixmap)
	self.labelAnimalRabbidFarm.hide()

	tmpPixmap = QtGui.QPixmap('images/animalCow.png')
	self.labelAnimalCowFarm.setPixmap(tmpPixmap)
	self.labelAnimalCowFarm.hide()

	tmpPixmap = QtGui.QPixmap('images/animalTiger.png')
	self.labelAnimalTigerFarm.setPixmap(tmpPixmap)
	self.labelAnimalTigerFarm.hide()




        # Load the file into a QMovie
        self.movie = QMovie("images/bonfire.gif")
 
        # Add the QMovie object to the label
        self.movie.setCacheMode(QMovie.CacheAll)

        self.labelFire1.setMovie(self.movie)
	self.labelFire2.setMovie(self.movie)
	self.labelFire3.setMovie(self.movie)
	self.labelFire4.setMovie(self.movie)
	self.labelFire5.setMovie(self.movie)
	self.labelFire6.setMovie(self.movie)

	self.labelFire1.setStyleSheet("background:transparent;")
	self.labelFire2.setStyleSheet("background:transparent;")
	self.labelFire3.setStyleSheet("background:transparent;")
	self.labelFire4.setStyleSheet("background:transparent;")
	self.labelFire5.setStyleSheet("background:transparent;")
	self.labelFire6.setStyleSheet("background:transparent;")
        self.movie.start()

	# Timer stuff	
	self.gameTimer = QtCore.QTimer()
	self.gameTimer.timeout.connect(self.gameTimerCallback)
	
	# Bind the event handlers
        self.btnExit.clicked.connect(self.btnExitClicked)  
        self.btnReset.clicked.connect(self.btnResetClicked)  
        self.btnStart.clicked.connect(self.btnStartClicked)  


	self.trigger.connect(self.handle_trigger)

    def handle_background_update(self):
	pixmap_bg = self.pixmap_bg.scaled(self.labelBackground.size(), QtCore.Qt.KeepAspectRatio)
	self.labelBackground.setPixmap(pixmap_bg)

    def handle_trigger(self, string):
	self.labelDrone.setPixmap(QtGui.QPixmap(string))


    def handle_trigger_farm(self, obj):
	obj.showFarm()

    def resetGame(self):
	self.score = 0
	self.lcdScore.display(self.score)

    def updateScore(self):
	self.score += 1
	self.lcdScore.display(self.score)

    def updateHighscore(self):
	if(self.highscore < self.score):
		self.highscore = self.score
		self.lcdHighscore.display(self.highscore)

    def gameTimerCallback(self):
	#print self.lcdTime.intValue()
	if (self.lcdTime.intValue()-1 < 0):
		self.gameTimer.stop()
		self.updateHighscore()
		self.resetGame()
	else:
		self.lcdTime.display(self.lcdTime.intValue()-1)

    def keyPressEvent(self, event):	
	key = event.key()
	if event.isAutoRepeat():
	    	if type(event) == QtGui.QKeyEvent and event.key() == QtCore.Qt.Key_A: 
			print "A"

	    	if type(event) == QtGui.QKeyEvent and event.key() == QtCore.Qt.Key_S:
			print "S"

    def btnExitClicked(self):    
	rospy.signal_shutdown("Exit program")
	QtGui.QApplication.quit()

    def btnResetClicked(self):
	self.gameTimer.stop()
	self.lcdTime.display(60)

    def btnStartClicked(self):
	self.resetGame()
	self.gameTimer.start(1000)

    def moveDrone(self, msg):
	if(msg.x>0 and msg.y>0):
		transfx = msg.y
		transfy = 300 - msg.x

		dronex = transfx*3 + 670
		droney = transfy*3 + 60

		droneWindowx = transfx*3
		droneWindowy = transfy*3

		self.labelDrone.move(dronex-20,droney-20)
		if self.CarryingAnimal == None:
			for animal in self.animalLabels:
				if not animal.isRescued():
					dist = sqrt((droneWindowx - animal.obj.x())**2 + (droneWindowy - animal.obj.y())**2)
					if dist < 30:
						animal.obj.setVisible(False)
						self.trigger.emit(animal.combinedImg)
						animal.setRescued(True)
						self.CarryingAnimal = animal
		else:
			if dronex > 1369 and droney < 261:
				print "Dropping of animal"
				self.trigger.emit("images/droneHighRes.png")
				self.updateScore()

				self.triggerFarm.emit(self.CarryingAnimal)
				self.CarryingAnimal = None

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
	#self.bgPixmap = QtGui.QPixmap.fromImage(qimg)
	self.pixmap_bg = QtGui.QPixmap.fromImage(qimg)
	self.triggerBackgroundUpdater.emit()
# Main
if __name__ == "__main__":
	# Load qt gui and stargame window
	app = QtGui.QApplication(sys.argv)
	gameWindow = StarGameWindow(None)

	# Init ROS
	rospy.init_node('starGameNode', anonymous=True)
	rospy.Subscriber("positionPublisher5", Point, gameWindow.pointCallback)
	rospy.Subscriber("imagePublisher", Image, gameWindow.imageCallback)
	# Start ros_spin in new thread since it is a blocking call
	threadROS = Thread(target = rosThread)
	threadROS.start()

	# Show game window
	gameWindow.show()
	#gameWindow.showFullScreen()
	app.exec_()


