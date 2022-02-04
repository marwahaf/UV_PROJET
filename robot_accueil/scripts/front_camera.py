import cv2
import sys
import math
from cv_bridge import CvBridge,CvBridgeError
import rospy, rospkg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from consts import *

#Get the path of the "Robot_accueil" package.
pkg_path = rospkg.RosPack().get_path("robot_accueil")
sys.path.append(pkg_path) 

model_path = pkg_path + "/dnn-model/MobileNetSSD_deploy"
dnn_model = model_path + ".caffemodel"
dnn_config = model_path + ".prototxt"

class Vision:
    def __init__(self, cvAnalyser, topic= "image_raw",depth_topic = "image_raw", synchronous= True, seconds= 0.1):
        # Attributs:
        self.bridge = CvBridge()
        self.cvAnalyser = cvAnalyser
        self.cv_image= None
        self.depths = None
        self.timer = None
        # Subsciber:
        self.depthSub = rospy.Subscriber(depth_topic,Image,self.depthCallback)
        if( synchronous ):
            self.subscriber = rospy.Subscriber(topic, Image, self.synchroCallback)
        else:
            self.subscriber = rospy.Subscriber(topic, Image, self.callback)
            self.timer = rospy.Timer( rospy.Duration(seconds), self.analyse)
        # Publisher:
        self.publisher = rospy.Publisher("analyzed/image_raw",Image, queue_size=10)
        self.person = rospy.Publisher("object",Point,queue_size=1)

    def callback(self, ros_image):
        self.cv_image= None
        try:
            self.cv_image= self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.cvAnalyser.read(self.cv_image)
        except CvBridgeError as e:
            print(e)

    def depthCallback(self, depths):
        self.depths = self.bridge.imgmsg_to_cv2(depths,desired_encoding="passthrough")

    def pixtoangle(self, size ,pix,cam_angle):
        ang = pix * cam_angle / size ##Calc angle from 0 to f.shape
        ang -= cam_angle/2 #Apply offset to set 0 in the middle
        #print("DBG : Angle = " + str(ang))
        #print("DBG : where x = "+ str(pix))
        #print("DBG : Shape" +str(f.shape))
        return math.radians(-ang) #-ang is for the Y values
    def analyse(self, data):
        if self.cv_image is not None :
            self.cvAnalyser.analyse()
            try:
                img,coords = self.cvAnalyser.result()
                self.publisher.publish(self.bridge.cv2_to_imgmsg(img, "bgr8") )
                #calculate the distances in the map
                xangle = self.pixtoangle(self.depths.shape[1],coords[0],CAMERA_ANGLE) #Get horizontal angle
                theta = self.pixtoangle(self.depths.shape[0],coords[1],CAMERA_ANGLE_OMEGA) #Get vertical angle
                d = (self.depths[coords[1]][coords[0]]/1000) * math.cos(theta)
                x = d * math.cos(xangle)
                y = d * math.sin(xangle)
                self.person.publish(Point(x,y,0))
            except CvBridgeError as e:
                print(e)

    def synchroCallback(self, data):
        self.callback(data)
        self.analyse(rospy.Time())

def decorateImage( img, lstLocs):
    iNbColors = len(tpColors)
    for box in lstLocs:
        ClassColor = (0,0,255)
        # boite englobante
        x1 = int(box[0])
        y1 = int(box[1])
        x2 = int(box[2])
        y2 = int(box[3])
        cv2.rectangle(img, (x1,y1,x2-x1,y2-y1), ClassColor,1)
        # nom de la classe
        cv2.putText(img, "Person", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, ClassColor, 2)

def findmiddle(x1,x2,y1,y2):
    if(x2 > x1 and y2 > y1):
        #good case
        return (int((x1+x2)/2),int((y1+y2)/2))
    else:
        print("Error finding middle : inverted rect")
        exit()


class Analyser:
    def __init__(self):
        
        print("Starting analyser") 
        self.model = cv2.dnn.readNet(model=dnn_model,config=dnn_config,framework='Caffe')
        print("Analyser Started")
    
    def __del__(self):
        print("End of Analyser")

    def read(self,cv_image):
        self.image=cv_image

    def analyse(self):
        # calcul des blobs : ATTENTION, il faut mettre l'image a la taille attendue par le DNN : 
        imgResized = cv2.resize(self.image, (iNET_INPUT_WIDTH, iNET_INPUT_HEIGHT), interpolation = cv2.INTER_AREA)
        blob = cv2.dnn.blobFromImage(image=imgResized, scalefactor=dbScaleFactor, size=(iNET_INPUT_WIDTH, iNET_INPUT_HEIGHT), mean=dbMeanVal)
        # necessaire pour recuperer les positions dans l'image initiale (avant redimensionnement)
        iImgWidth = self.image.shape[1]
        iImgHeight = self.image.shape[0]
        # on les "blobs" de l'image a l'entree du classifieur 
        self.model.setInput(blob)
        # declenchement de l'inference
        outputs = self.model.forward()
        final_outputs = outputs[0]
        # remise en forme des sorties
        final_outputs = final_outputs.reshape(iOUTPUT_NB_LINES, iOUTPUT_NB_COLS)
        #....................................................................
        # affichage des infos relatives aux objets detectes, le cas echeant : 
        # ................................................................... 
        iSize = final_outputs.shape
        lstLabels = []
        lstNames = []
        lstLocalisation = []
        lstProba = []
        for i in range(iSize[0]):
            # parcours des lignes
            vLine = final_outputs[i,:]
            # quelque chose ?
            if vLine[iCLASS_CONFIDENCE] > 0.0:
                # oui : 
                iClass = int(vLine[iCLASS_ID])
                #une personne ?
                if(lstszClassName[iClass] == 'person'):
                    lstLabels.append(iClass)
                    lstProba.append(vLine[iCLASS_CONFIDENCE])
                    lstNames.append(lstszClassName[iClass])
                    if(vLine[iCLASS_CONFIDENCE]>= 0.95):
                        x1 = round(vLine[iX_TOPLEFT] * iImgWidth   )
                        y1 = round(vLine[iY_TOPLEFT] * iImgHeight  )
                        x2 = round(vLine[iX_LOWRIGHT] * iImgWidth  )
                        y2 = round(vLine[iY_LOWRIGHT] * iImgHeight )
                        self.personCoords = findmiddle(x1,x2,y1,y2)
                        cv2.circle(self.image, findmiddle(x1,x2,y1,y2) , 5, (255,0,0), 10)
                        lstLocalisation.append([x1,y1,x2,y2])

        # affichage texte des informations de detection : 
        print("objets detectes = ")
        print(lstNames)
        print("localisations = ")
        print(lstLocalisation)
        print("probabilites reseau = ")
        print(lstProba)
        # affichage des elements detectes dans la fenetre graphique
        decorateImage( self.image, lstLocalisation)
        #cv2.imshow('DETECTION', self.image)
        #cv2.waitKey(2)

    def result(self):
        return self.image, self.personCoords

if __name__=='__main__':
    print("> front_camera start")
    rospy.init_node("vision", anonymous=True)
    camera = Vision(
        Analyser(),
        topic ="camera/color/image_raw",
        depth_topic ="camera/aligned_depth_to_color/image_raw",
        synchronous=False)
    rospy.spin()
    cv2.destroyAllWindows()