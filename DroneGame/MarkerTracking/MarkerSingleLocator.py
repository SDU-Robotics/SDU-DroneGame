#!/usr/bin/env python
from time import time, strftime
import sys
import os

sys.path.append('/opt/ros/hydro/lib/python2.7/dist-packages')
import cv
import math
import signal

from ImageAnalyzer import ImageAnalyzer
from TrackerInWindowMode import TrackerInWindowMode
from PerspectiveTransform import PerspectiveCorrecter
from MarkerPose import MarkerPose

'''
2012-10-10
Script developed by Henrik Skov Midtiby (henrikmidtiby@gmail.com).
Provided for free but use at your own risk.

2013-02-13 
Structural changes allows simultaneous tracking of several markers.
Frederik Hagelskjaer added code to publish marker locations to ROS.
'''

PublishToROS = True

if PublishToROS:
    #import roslib; 
    #roslib.load_manifest('frobitLocator')
    import rospy
    from geometry_msgs.msg import Point
            



class CameraDriver:
    ''' 
    Purpose: capture images from a camera and delegate procesing of the 
    images to a different class.
    '''
    def __init__(self, markerOrders = 5, defaultKernelSize = 21, scalingParameter = 2500, cameraNumber = 0):
        # Initialize camera driver.
        # Open output window.
        cv.NamedWindow('filterdemo', cv.CV_WINDOW_AUTOSIZE)

        self.setFocus(cameraNumber)
        self.camera = cv.CaptureFromCAM(cameraNumber)
        self.setResolution()
	
	self.cropCounter = 0

        # Storage for image processing.
        self.currentFrame = None
        self.processedFrame = None
        self.running = True
        # Storage for trackers.
        self.trackers = None
        self.windowedTrackers = None
        self.oldLocations = None

        # Initialize trackers.
    	self.trackers = ImageAnalyzer(downscaleFactor=1)
    	self.trackers.addMarkerToTrack(markerOrder, defaultKernelSize, scalingParameter)
    	self.windowedTrackers = TrackerInWindowMode(markerOrder, defaultKernelSize)
    	self.oldLocations = MarkerPose(None, None, None, None)
        self.cnt = 0
        self.defaultOrientation = 0

    def setFocus(self,cameraNumber):
        # Disable autofocus
        os.system('v4l2-ctl -d ' + str(cameraNumber) + ' -c focus_auto=0')
        
        # Set focus to a specific value. High values for nearby objects and
        # low values for distant objects.
        os.system('v4l2-ctl -d ' + str(cameraNumber) + ' -c focus_absolute=0')

        # sharpness (int)    : min=0 max=255 step=1 default=128 value=128
        os.system('v4l2-ctl -d ' + str(cameraNumber) + ' -c sharpness=200')

    def setResolution(self):
        cv.SetCaptureProperty(self.camera, cv.CV_CAP_PROP_FRAME_WIDTH, 1280)
        cv.SetCaptureProperty(self.camera, cv.CV_CAP_PROP_FRAME_HEIGHT, 720)

    def getImage(self):
        # Get image from camera.
        self.currentFrame = cv.QueryFrame(self.camera)
        
    def processFrame(self):
        # Locate all markers in image.
            #if(self.oldLocations[k].x is None or self.cropCounter == 30):
                # Previous marker location is unknown, search in the entire image.
                self.processedFrame = self.trackers[k].analyzeImage(self.currentFrame)
                markerX = self.trackers[k].markerLocationsX[0]
                markerY = self.trackers[k].markerLocationsY[0]
                self.oldLocations = MarkerPose(markerX, markerY, 0, 0, 0)
		self.cropCounter = 0
            #else:
                # Search for marker around the old location.
            #    self.windowedTrackers[k].cropFrame(self.currentFrame, self.oldLocations[k].x, self.oldLocations[k].y)
            #    self.oldLocations[k] = self.windowedTrackers[k].locateMarker()
            #    self.windowedTrackers[k].showCroppedImage()
	#	self.cropCounter += 1
    
    def drawDetectedMarkers(self):
        for k in range(len(self.trackers)):
            xm = self.oldLocations[k].x
            ym = self.oldLocations[k].y
            orientation = self.oldLocations[k].theta
            cv.Circle(self.processedFrame, (xm, ym), 4, (55, 55, 255), 2)
            xm2 = int(xm + 50*math.cos(orientation))
            ym2 = int(ym + 50*math.sin(orientation))
            cv.Line(self.processedFrame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)

    
    def showProcessedFrame(self):
        cv.ShowImage('filterdemo', self.processedFrame)
        pass

    def resetAllLocations(self):
        # Reset all markers locations, forcing a full search on the next iteration.
        for k in range(len(self.trackers)):
            self.oldLocations[k] = MarkerPose(None, None, None, None)

        
    def handleKeyboardEvents(self):
        # Listen for keyboard events and take relevant actions.
        key = cv.WaitKey(20) 
        # Discard higher order bit, http://permalink.gmane.org/gmane.comp.lib.opencv.devel/410
        key = key & 0xff
        if key == 27: # Esc
            self.running = False
        if key == 114: # R
            print("Resetting")
            self.resetAllLocations()
        if key == 115: # S
            # save image
            print("Saving image")
            filename = strftime("%Y-%m-%d %H-%M-%S")
            cv.SaveImage("output/%s.png" % filename, self.currentFrame)

    def returnPositions(self):
        # Return list of all marker locations.
        return self.oldLocations
 
    def signal_handler(self, signal, frame):
	self.running = False

class RosPublisher:
    def __init__(self, markers):
        # Instantiate ros publisher with information about the markers that 
        # will be tracked.
        self.pub = []
        self.markers = markers
        for i in markers:
            self.pub.append( rospy.Publisher('positionPublisher' + str(i), Point)  )       
        rospy.init_node('DroneLocator')   

    def publishMarkerLocations(self, locations):
        j = 0        
        #for i in self.markers:
           # print 'x%i %i  y%i %i  o%i %i' %(i, locations[j][0], i, locations[j][1], i, locations[j][2])
            #ros function        
        self.pub[j].publish(  Point( locations.x, locations.y, 0 )  )
        j = j + 1                
        


def main():
    toFind = [5]

    if PublishToROS:  
        RP = RosPublisher(toFind)
       
    cd = CameraDriver(toFind, defaultKernelSize = 15, cameraNumber = 0) 
     
    signal.signal(signal.SIGINT, cd.signal_handler)
    pointLocationsInImage = [[320, 0], [1032, 0], [1025, 712], [327, 705]]
    realCoordinates = [[0, 0], [300, 0], [300, 300], [0, 300]]
    perspectiveConverter = PerspectiveCorrecter(pointLocationsInImage, realCoordinates)
 
    while cd.running:
        cd.getImage()
        timestamp = time()
        cd.processFrame()
	if not PublishToROS:
        	cd.drawDetectedMarkers()
	        cd.showProcessedFrame()
	        cd.handleKeyboardEvents()
        y = cd.returnPositions()     
        if PublishToROS:
            RP.publishMarkerLocations(perspectiveConverter.convertPose(y[0]))
        else:
            pass
            #print y
            for k in range(len(y)):
                try:
		    if not PublishToROS:
                    	poseCorrected = perspectiveConverter.convertPose(y[k])
	                print "x: ", poseCorrected.x, " y: ", poseCorrected.y
                    #print("%8.3f %8.3f" % (poseCorrected.x, poseCorrected.y))
		    #print "x: ", poseCorrected.x, " y: ", poseCorrected.y, " timestamp: ", timestamp
                except:
                    pass      
            
    print("Stopping")
main()
