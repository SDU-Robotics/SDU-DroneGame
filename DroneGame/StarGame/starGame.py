# Imports
import sys
import rospy
from PyQt4 import QtCore, QtGui, uic
from threading import Thread
from geometry_msgs.msg import Point
from random import randint

# Load GUI
formClass = uic.loadUiType("starGameUI.ui")[0]               

useROS = False

# Ros spin thread
def rosThread():
	rospy.spin()
	
# My QT class
class StarGameWindow(QtGui.QMainWindow, formClass):
    def __init__(self, parent=None):
	# Setup QT gui
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)

	# Variables	
	self.gameLength = 60 # seconds
	self.score = 0
	self.highscore = 0
	self.updateScore()
	self.xOffset = 610
	self.yOffset = 0
	self.xSize = 600
	self.ySize = 600	
	self.currentPos = Point(0,0,0)
	self.oldPos = Point(1920/2,1080/2,0)
	self.starPos = Point(0,0,0)	

	# Setup logo
	logoPixmap = QtGui.QPixmap('images/sdulogo.png')
	logoScaledPixmap = logoPixmap.scaled(logoPixmap.width()/1.7,logoPixmap.height()/1.7, QtCore.Qt.KeepAspectRatio)
	self.labelLogo.setPixmap(logoScaledPixmap)

	# Setup bagground
	bgPixmap = QtGui.QPixmap('images/bg.png')
	bgScaledPixmap = bgPixmap.scaled(900,900, QtCore.Qt.KeepAspectRatio)
	self.labelBackground.setPixmap(bgScaledPixmap)

	# Load star image.
	starPixmap = QtGui.QPixmap('images/apple.png')
	starScaledPixmap = starPixmap.scaled(50,50, QtCore.Qt.KeepAspectRatio)
	self.labelStar.setPixmap(starScaledPixmap)
	self.labelStar.move(1920/2,1080/4)
	self.labelStar.setVisible(False)

	# Load drone image.
	dronePixmap = QtGui.QPixmap('images/droneHighResRotated.png')
	droneScaledPixmap = dronePixmap.scaled(90,90, QtCore.Qt.KeepAspectRatio)
	self.labelDrone.setPixmap(droneScaledPixmap)
	self.labelDrone.move(1920/2,1080/2)

	# Progress bar
	self.barTime.setRange(0,self.gameLength)
	self.barTime.setValue(self.gameLength)
	self.barTime.setTextVisible(False)

	# Timer stuff	
	self.gameTimer = QtCore.QTimer()
	self.gameTimer.timeout.connect(self.gameTimerCallback)
	self.starTimer = QtCore.QTimer()
	self.starTimer.timeout.connect(self.starTimerCallback)

	# Bind the event handlers
        self.btnExit.clicked.connect(self.btnExitClicked)  
        self.btnReset.clicked.connect(self.btnResetClicked)  
        self.btnStart.clicked.connect(self.btnStartClicked)  

    def generateStar(self):
	self.starPos.x = randint(self.xOffset,self.xOffset+self.xSize)
	self.starPos.y = randint(self.yOffset,self.yOffset+self.ySize)
	self.labelStar.move(self.starPos.x,self.starPos.y)

    def updateScore(self):
	self.score += 1
	self.lcdScore.display(self.score)

    def updateHighscore(self):
	if(self.highscore < self.score):
		self.highscore = self.score
		self.lcdHighscore.display(self.highscore)

    def starTimerCallback(self):
	if useROS == True:
		if(abs(self.starPos.x-self.currentPos.x)<60 and abs(self.starPos.y-self.currentPos.y)<60):
			self.updateScore()
			self.generateStar()
	elif(abs(self.starPos.x-self.currentPos.x-600)<60 and abs(self.starPos.y-self.currentPos.y)<60):
		self.updateScore()
		self.generateStar()

    def gameTimerCallback(self):
	if(self.barTime.value()-1 < 0):
		self.updateHighscore()
		self.resetGame()
	else:
		self.barTime.setValue(self.barTime.value()-1)
		self.labelTime.setText(QtCore.QString.number(self.barTime.value()))

    def keyPressEvent(self, event):
	# Manual control
	movePixel = 20	
	key = event.key()
    	if key == QtCore.Qt.Key_W:  
		self.currentPos.y = self.oldPos.y - movePixel
	if key == QtCore.Qt.Key_D:  
		self.currentPos.x = self.oldPos.x + movePixel
	if key == QtCore.Qt.Key_A:  
		self.currentPos.x = self.oldPos.x - movePixel
	if key == QtCore.Qt.Key_S:  
		self.currentPos.y = self.oldPos.y + movePixel
	
	if(self.currentPos.y>=0 and self.currentPos.x>=0):
		self.oldPos = self.currentPos
		self.moveDrone(self.currentPos)

    def btnExitClicked(self):    
	rospy.signal_shutdown("Exit program")
	QtGui.QApplication.quit()

    def btnResetClicked(self):  
	self.resetGame()

    def resetGame(self):
	self.starTimer.stop()
	self.gameTimer.stop()
	self.labelStar.setVisible(False)

    def btnStartClicked(self):  
	self.score = 0
	self.lcdScore.display(self.score) 
	self.labelTime.setText(QtCore.QString.number(self.gameLength))
	self.barTime.setValue(self.gameLength)
	self.generateStar()
	self.labelStar.setVisible(True)
	self.gameTimer.start(1000)
	self.starTimer.start(100)

    def moveDrone(self, msg):
	if useROS == True:
		self.labelDrone.move(self.xOffset + msg.y*3, self.yOffset + 900-msg.x*3)
		self.oldPos.x = self.xOffset+msg.y*3
		self.oldPos.y = self.yOffset+900-msg.x*3
		self.currentPos = self.oldPos 
	else:
		self.labelDrone.move(self.xOffset + msg.x, self.yOffset + msg.y)

    def pointCallback(self, msg):
	self.moveDrone(msg)

# Main
if __name__ == "__main__":
	# Load qt gui and stargame window
	app = QtGui.QApplication(sys.argv)
	gameWindow = StarGameWindow(None)

	if useROS == True:
		# Init ROS
		rospy.init_node('starGameNode', anonymous=True)
		rospy.Subscriber("positionPublisher5", Point, gameWindow.pointCallback)

		# Start ros_spin in new thread since it is a blocking call
		threadROS = Thread(target = rosThread)
		threadROS.start()

	# Show game window
	gameWindow.show()
	gameWindow.showFullScreen()
	app.exec_()


