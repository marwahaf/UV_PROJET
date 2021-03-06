#!/usr/bin/python3
import cv2
import sys
import math
from cv_bridge import CvBridge,CvBridgeError
import rospy, rospkg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Pose,Quaternion, Point,PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

#Get the path of the "Robot_accueil" package.
pkg_path = rospkg.RosPack().get_path("robot_accueil")
sys.path.append(pkg_path) 

model_path = pkg_path + "/dnn-model/MobileNetSSD_deploy"
dnn_model = model_path + ".caffemodel"
dnn_config = model_path + ".prototxt"

CAMERA_ANGLE= 69
CAMERA_ANGLE_OMEGA = 42
""" Modele DNN """

dbMeanVal = 127.5                                       # ->utilise pour la fonction de construction des
dbScaleFactor = 0.007843                                #   "blob" que necessite l'utilisation de ce DNN
# indice pour decoder la sortie du reseau               #    avec OpenCV
iCLASS_ID = 1                                           # ->indice correspondant a l'identifiant (entier) de la classe
iCLASS_CONFIDENCE = 2                                   # ->indice correspondant a la probabilit√© d'appartenance a la classe
                                                        #   (selon le "point de vue", parfois discutable, du reseau lui-meme...)
iX_TOPLEFT = 3                                          # ->indice de l'abscisse du coin superieur gauche de la fenetre de detection d'un objet
iY_TOPLEFT = 4                                          # ->indice de l'ordonnee....
iX_LOWRIGHT = 5                                         # ->indice de l'abscisse du coin inferieur droit de la fenetre de detecton d'un objet
iY_LOWRIGHT = 6                                         # ->indice de l'ordonnee
# taille des images presentees au reseau
iNET_INPUT_WIDTH = 300                                  # ->pour se conformer a la taille des donnees d'entre du reseau DNN
iNET_INPUT_HEIGHT = 300                                 #   (idem)
# autres parametres
iOUTPUT_NB_LINES = 100                                  # ->dimension de la sortie du DNN
iOUTPUT_NB_COLS = 7                                     # ->dimension de la sortie du DNN
#..................................
# couleurs pour affichage graphique
#..................................
tpColors= [ (0,0,255), (0,255,0), (0,255,255),(255,0,0),(255,0,255),(255,255,0),(255,255,255) ]

#.....................................................
# liste des classes prises en compte dans ce modele: :
#..................................................... 
lstszClassName = ("background","aeroplane", "bicycle",
    "bird", "boat","bottle", "bus", "car", "cat", "chair",
    "cow", "diningtable", "dog", "horse","motorbike", "person",
    "pottedplant", "sheep", "sofa", "train", "tvmonitor")


# Class link between ROS and DNN-Model apply on camera
class Vision:
    def __init__(self, cvAnalyser, topic= "image_raw",depth_topic = "image_raw", synchronous= True, seconds= 0.1):
        # Attributs:
        self.bridge = CvBridge()
        self.cvAnalyser = cvAnalyser
        self.cv_image= None
        self.depths = None
        self.timer = None
        self.countFrames= 0
        self.start = True

        # Subscibers:
        self.homegoal = rospy.Subscriber('/goal/home_returned', PoseStamped, self.returnhome)   # When topic is sent to this node, some vars are reset
        self.depthSub = rospy.Subscriber(depth_topic,Image,self.depthCallback)      # Subscribing to the "aligned_rgb_to_depth" image
        
        # If we want synchronous object detection and getting image camera
        # If too many images are sent, program will be very slow, maybe using it as asynchronous get better perfs.
        if( synchronous ):
            self.subscriber = rospy.Subscriber(topic, Image, self.synchroCallback)
        else:
            self.subscriber = rospy.Subscriber(topic, Image, self.callback)
            self.timer = rospy.Timer( rospy.Duration(seconds), self.analyse)

        # Publisher:
        self.publisher = rospy.Publisher("analyzed/image_raw",Image, queue_size=10)     # Publish the analysed image to show object detected
        self.person = rospy.Publisher("/person",Marker,queue_size=1)    # Marker with coordinates of the object detected
        self.goal_person = rospy.Publisher("/goal/person",PoseStamped,queue_size=1)     # Goal for the robot moving with the first coords of the object detected

    """ Callback method : Get the ROS image from camera and transform it to a opencv-readable image 
        and stores it as an attribute to be reused for processing with different methods.
    """
    def callback(self, ros_image):
        self.cv_image= None
        try:
            self.cv_image= self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.cvAnalyser.read(self.cv_image)
        except CvBridgeError as e:
            print(e)

    """ depthCallback method : Do like "callback" method, but with the "depths" image got from camera."""
    def depthCallback(self, depths):
        self.depths = self.bridge.imgmsg_to_cv2(depths,desired_encoding="passthrough")

    """ pixtoangle method: 
        convert a pixel position to the real angle of the object pointed in the camera frame
    """
    def pixtoangle(self, size ,pix,cam_angle):
        ang = pix * cam_angle / size    # Calc angle from 0 to f.shape
        ang -= cam_angle/2 # Apply offset to set 0 in the middle
        return math.radians(-ang) #-ang is because positives Y are left and not right

    """ returnhome method:
        this is a callback method to reset the goal-sending procedure when starting the robot
    """
    def returnhome(self, data):
        self.start = True
        print('Returned home, camera OK.')

    """ analyse method :
        uses the Analyser class to get coords of the object and calculates position in the map frame
        an then send markers or goals.
    """
    def analyse(self, data):
        if self.cv_image is not None and self.depths is not None :
            self.cvAnalyser.analyse()
            try:
                img,coords = self.cvAnalyser.result()   # Get the resulted image and the coords of the detected object
                self.publisher.publish(self.bridge.cv2_to_imgmsg(img, "bgr8") )     # publish the image to be read by Rviz or rqt_image_view
                
                # if there is an detected object and the distance is relevant (not 0)
                if(coords[0]!=-1 and coords[1]!=-1) and self.depths[coords[1]][coords[0]]!=0:
                    xangle = self.pixtoangle(self.depths.shape[1],coords[0],CAMERA_ANGLE) #Get horizontal angle
                    theta = self.pixtoangle(self.depths.shape[0],coords[1],CAMERA_ANGLE_OMEGA) #Get vertical angle
                    d = (self.depths[coords[1]][coords[0]]/1000) * math.cos(theta)  #Get the projection of the distance in a 2D frame
                    x = d * math.cos(xangle)    #Calculate X coord in a 2D frame
                    y = d * math.sin(xangle)    #Calculate Y coord in a 2D frame
                    person = Marker(            #Create a Cylinder marker with the coordinates of the object (Can be upgradable with the size of the object)
                        type = Marker.CYLINDER,
                        lifetime = rospy.Duration(0),
                        scale=Vector3(0.1, 0.1, 0.1),
                        color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                        pose = Pose(Point(x,y,0),Quaternion())
                        )
                    person.header.frame_id = 'laserback_link'   #Setting the marker in "laserback_link" frame because there is only a Z translation between laser back and camera
                    print(x,y,d, self.depths[coords[1]][coords[0]], coords[1],coords[0])    # Print values for debug
                    
                    #if we have 10 successive frames with the object
                    if self.countFrames>= 10:
                        self.person.publish(person)     # publish topic with person
                        print("Pub person")     #print for debug
                        #if this is the starting of the robot procedure
                        if self.start:
                            #Create a goal only one time
                            print("pub goal_person")
                            self.start = False
                            tmp_goal = PoseStamped()
                            tmp_goal.header.frame_id = 'laserback_link'
                            tmp_goal.pose.position.x = x
                            tmp_goal.pose.position.y = y
                            self.goal_person.publish(tmp_goal)
                    #if there is less than 10 successives frames with the object
                    else:
                        self.countFrames +=1
                #if there is no object detected or the distance is not relevant
                else:
                    #if there is object
                    if(coords[0]!=-1 and coords[1]!=-1):
                        self.countFrames = 0
            except CvBridgeError as e:
                print(e)

    """ synchroCallback method:
        gets the image and the processing in the same time.
    """
    def synchroCallback(self, data):
        self.callback(data)
        self.analyse(rospy.Time())

""" decorateImage function :
    decorate an image when getting localization of rects
"""
def decorateImage( img, lstLocs):
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

""" findmiddle function : 
    finds the middle of a rect
    function exists to have the center of the object and get the coords.
    the "getting distance of the object procedure" can be upgraded.
"""
def findmiddle(x1,x2,y1,y2):
    if(x2 > x1 and y2 > y1):
        #good case
        return (int((x1+x2)/2),int((y1+y2)/2))
    else:
        print("Error finding middle : inverted rect")

# Class analyser: reads the dnn model and applies it to detect objects in images
class Analyser:

    def __init__(self): # reads the dnn model.
        
        print("Starting analyser") 
        self.model = cv2.dnn.readNet(model=dnn_model,config=dnn_config,framework='Caffe')
        print("Analyser Started")
    
    def __del__(self):  #when programm is ended
        print("End of Analyser")


    def read(self,cv_image):
        #store the image as attribut
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
        self.personCoords = (-1,-1)
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
                    #Si probabilit√© r√©seau > √† seuil
                    if(vLine[iCLASS_CONFIDENCE]>= 0.7):
                        x1 = round(vLine[iX_TOPLEFT] * iImgWidth   )
                        y1 = round(vLine[iY_TOPLEFT] * iImgHeight  )
                        x2 = round(vLine[iX_LOWRIGHT] * iImgWidth  )
                        y2 = round(vLine[iY_LOWRIGHT] * iImgHeight )
                        self.personCoords = findmiddle(x1,x2,y1,y2)
                        cv2.circle(self.image, self.personCoords , 5, (255,0,0), 10)
                        lstLocalisation.append([x1,y1,x2,y2])

        # affichage texte des informations de detection : 
        decorateImage( self.image, lstLocalisation)


    def result(self):
        #returns the image and the coords of the object.
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