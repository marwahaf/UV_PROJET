
CAMERA_ANGLE= 69
CAMERA_ANGLE_OMEGA = 42

""" Modele DNN """

dbMeanVal = 127.5                                       # ->utilise pour la fonction de construction des
dbScaleFactor = 0.007843                                #   "blob" que necessite l'utilisation de ce DNN
# indice pour decoder la sortie du reseau               #    avec OpenCV
iCLASS_ID = 1                                           # ->indice correspondant a l'identifiant (entier) de la classe
iCLASS_CONFIDENCE = 2                                   # ->indice correspondant a la probabilité d'appartenance a la classe
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
