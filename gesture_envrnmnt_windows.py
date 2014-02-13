import cv2
import time,os,threading
import numpy as np
from numpy import sqrt,arccos,rad2deg
import math
import win32api as win32
import win32con

cam = cv2.VideoCapture(1)

'''class declaration '''

class hand:
''' Fucntions to accept the filter values and store in dictionary Vars '''
    def onChange_fuS(self,value):
        self.Vars["filterUpS"] = value
        #pickle.dump(Vars, open(".config", "w"))
    def onChange_fdS(self,value):
        self.Vars["filterDownS"] = value
        #pickle.dump(Vars, open(".config", "w"))

    def onChange_fuV(self,value):
        self.Vars["filterUpV"] = value
        #pickle.dump(Vars, open(".config", "w"))

    def onChange_fdV(self,value):
        self.Vars["filterDownV"] = value
        #pickle.dump(self.Vars, open(".config", "w"))
                            

    def onChange_upper(self,value):
        self.Vars["upper"] = value
        #pickle.dump(Vars, open(".config", "w"))
    def onChange_lower(self,value):
        self.Vars["lower"] = value
        #pickle.dump(Vars, open(".config", "w"))
    def onChange_erode(self,value):
        self.Vars["erode"] = value + 1
        #pickle.dump(Vars, open(".config", "w"))
                            
   
                            
    def onChange_dilate(self,value):
        self.Vars["dilate"] = value + 1
        #pickle.dump(Vars, open(".config", "w"))
                            
                       
    def onChange_smooth(self,value):
        self.Vars["smooth"] = value + 1
        #pickle.dump(Vars, open(".config", "w"))
    def __init__(self):
        self.start = 0
        self.Vars = {"filterUpS":255,
                                   "filterDownS":89,
                                   "filterUpV":255,
                                   "filterDownV":53,
                                   "upper":255,
                                   "lower":141,
                                   "erode":1,
                                   "dilate":4,
                                   "smooth":4}
        self.data = {"area":0,
                          "hulls":0,
                          "defects":0,
                          "cursor":(0,0),
                          "defects":0,
                          "angles less 90":0,
                         "fingers": 0,
                         "fingers history": [0]}

''' Tackbars for hsv range '''
        cv2.namedWindow("Filters")
        cv2.createTrackbar("erode", "Filters", self.Vars["erode"], 255, self.onChange_erode)
        cv2.createTrackbar("dilate", "Filters", self.Vars["dilate"], 255, self.onChange_dilate)
        cv2.createTrackbar("smooth", "Filters", self.Vars["smooth"], 255, self.onChange_smooth)

        cv2.namedWindow("HSV Filters")        
        cv2.createTrackbar("upper", "HSV Filters", self.Vars["upper"], 255, self.onChange_upper)
        cv2.createTrackbar("filterUpS", "HSV Filters", self.Vars["filterUpS"], 255, self.onChange_fuS)
        cv2.createTrackbar("filterUpV", "HSV Filters", self.Vars["filterUpV"], 255, self.onChange_fuV)        
        cv2.createTrackbar("lower", "HSV Filters", self.Vars["lower"], 255, self.onChange_lower)   
        cv2.createTrackbar("filterDownS", "HSV Filters", self.Vars["filterDownS"], 255, self.onChange_fdS)
        cv2.createTrackbar("filterDownV", "HSV Filters", self.Vars["filterDownV"], 255, self.onChange_fdV)
         
        while True:
            self.camera_detection()
            print "I am just before the loop start = ", self.start
            
            if self.start == 1:
                print "calling mouse"
                self.mouse()
            key = cv2.waitKey(10)
            if key == 27:
                exit(0)
    
'''fucntion to find distance b/w two points '''
    def distance(self,point1,point2):
        x = abs(point1[0] - point2[0])
        y = abs(point1[1] -point2[1])
        d = sqrt(x**2+y**2)
        return d
''' mouse movement and initiation of events . threading should be implemented not done here now '''
    def mouse(self):
        print "entered mouse" 
        x = self.data["cursor"][0]
        y = self.data["cursor"][1]
        x = 2.15 *x
        y = 2.7 * y
        x = round(x)
        y = round(y)
        x = math.trunc(x)
        y = math.trunc(y)
        
        win32.SetCursorPos((x,y))
'''function doing skin filtering and finding hand using convex hull and calculating moments and storing moment into data["cursor"] dictionary '''
    def camera_detection(self):
        cam.set(3,640)
        cam.set(4,480)
        ret,img1 = cam.read()
        im = img1
        hsv = cv2.cvtColor(img1,cv2.COLOR_BGR2HSV)
        UPPER = np.array([self.Vars["upper"], self.Vars["filterUpS"], self.Vars["filterUpV"]], np.uint8)
        LOWER = np.array([self.Vars["lower"], self.Vars["filterDownS"], self.Vars["filterDownV"]], np.uint8)
        bin_im = cv2.inRange(hsv,LOWER, UPPER)
        
        #ret,thresh = cv2.threshold(bin_im,15,255,0)
        e = self.Vars["erode"]
        d = self.Vars["dilate"]
        s = self.Vars["smooth"]
        print e,d,s
        
        bin_im = cv2.erode(bin_im,
                           cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(e, e)))     
        #eroded = cv2.erode(thresh,element)
        #cv2.imshow('Erode',eroded)
        bin_im = cv2.dilate(bin_im,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(d,d)))
        bin_im = cv2.blur(bin_im, (s, s))
        cv2.imshow('Binary Image',bin_im)
        #dilated = cv2.dilate(eroded,element)
        #cv2.imshow('True Binary',dilated)
        #ret,thresh = cv2.threshold(dilated,15,255,0)
        #cv2.imshow('Threshold',thresh)
        contours, hierarchy = cv2.findContours(bin_im,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            #cv2.drawContours(im,contours,-1,(0,255,0),3)

            #eliminating small areas
        allIdex = []
        for index in range(len(contours)):
            area = cv2.contourArea(contours[index])
            if area < 5e3: allIdex.append(index)
        allIdex.sort(reverse=True)
        for index in allIdex: contours.pop(index)
            #if no contour stopping 
            #if len(contours) == 0:return

        allIdex = []
        index_ = 0
            
        for cnt in contours:
            data["area"] =  cv2.contourArea(cnt)
            area=cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True),True)
            hull = cv2.convexHull(cnt)
            last = None
            data["hulls"]+=1
            for hu in hull:
                if last == None:cv2.circle(im,tuple(hu[0]),10,(0,0,255),5)
                else:
                    distan =  self.distance(last,tuple(hu[0]))
                    if distan > 40:
                        data["hulls"] +=1
                        cv2.circle(im,tuple(hu[0]),10,(0,0,255),5)
                last = tuple(hu[0])

                #calculating moment
                    
            moment = cv2.moments(cnt)
            centroid_x = int(moment['m10']/moment['m00'])
            centroid_y = int(moment['m01']/moment['m00'])
            cv2.circle(im,(centroid_x,centroid_y),20,(0,255,255),10)
            self.data["cursor"] = (centroid_x,centroid_y)
            print "moment",centroid_x,centroid_y
                #print data["cursor"]
            cv2.drawContours(im,contours,-1,(64,255,85),-1)
               # defects
            hull = cv2.convexHull(cnt,returnPoints=False)
            angles = []
            defects = cv2.convexityDefects(cnt,hull)
            if defects == None:return

            data["defects"] = 0
            for i in range(defects.shape[0]):
                s,e,f,d = defects[i,0]
                if d>1000:
                    start = tuple(cnt[s][0])
                    end = tuple(cnt[e][0])
                    far = tuple(cnt[f][0])
                    data["defects"]+=1
                    cv2.circle(im,far,5,[0,255,255],-1)
                    cv2.line(im, start, far, [255, 0, 0], 5) 
                    cv2.line(im, far, end, [255, 0, 0], 5)
                    #angles.append(angle(self,far, start, end))
            b = filter(lambda a:a<90, angles)
            data["angles less 90"] = len(b)
            data["fingers"] = len(b) + 1
            data["fingers history"].append(len(b) + 1)
            print["fingers"]
            
           
            key = cv2.waitKey(10)
            if key == ord(' '):
                self.start = 1
                print "entered start"
                
                #arr = x,y
                #print "mouse",arr
                '''win32.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN,0,0)
                win32.mouse_event(win32con.MOUSEEVENTF_LEFTUP,0,0)
                win32.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN,0,0)
                win32.mouse_event(win32con.MOUSEEVENTF_LEFTUP,0,0)'''
            cv2.imshow('imae',im)
  #'''def angle(self,cent, rect1, rect2):
   #   v1 = (rect1[0] - cent[0], rect1[1] - cent[1])
    #  v2 = (rect2[0] - cent[0], rect2[1] - cent[1])
     # dist = lambda a:sqrt(a[0] ** 2 + a[1] ** 2)
      #angle = arccos((sum(map(lambda a, b:a*b, v1, v2))) / (dist(v1) * dist(v2)))
      #angle = abs(rad2deg(angle))
      #return angle'''
                        


if __name__ == "__main__":
    hand()
    #camera_detection()
    
    
            
        
        
        
        
    
