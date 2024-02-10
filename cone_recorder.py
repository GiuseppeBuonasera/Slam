import rospy
import math
from geometry_msgs.msg import PoseStamped , Point
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray

errore = 0.5
count=0
conteggio_punti = {} 
punti_filtrati = []
buffer = []

# modificare il codice in modo tale da guardare per prima la lista definitiva



def callback(data):
    global count
    punti_circo = []
    
    x = data.x
    y = data.y
    
    point = x, y
    #gestisco i punti nulli
    if point == (0,0):
        point = None
    else:
        buffer.append(point)
    
    
    for valori in buffer: 
           
        filtra_punti(valori, errore)

    #quando il counter va a 20 pulisco le liste     
    if count == 30:   
        conteggio_punti.clear()
        punti_circo.clear()
        buffer.clear()

        count = 0
    else:
        count +=1
    
   
    #pubblicazione cerchi su rviz
    #for a in punti_filtrati:
        #punti_cerchi = circonferenza(a, r=0.5, numero_punti=360)
        #punti_circo.append(punti_cerchi)
    publish_cylinders(punti_filtrati)

    print(punti_filtrati)
    
    




def lista_conteggio_punti(punti):
    x,y = punti
    
    trovato = False
    for (x1, y1), conteggio in conteggio_punti.items():
        differenza_x = abs(x - x1)
        differenza_y = abs(y - y1)
        condizione = differenza_x <= 0.5 and differenza_y <= 0.5
        if condizione:
            xm = (x + x1)/2
            ym = (y + y1)/2
            conteggio_punti[x1,y1] = conteggio_punti.get((x1,y1), 0) + 1
            conteggio_punti[xm, ym] = conteggio_punti.pop((x1, y1))
            trovato = True
            break
    if not trovato:
        conteggio_punti[(x, y)] = 1

    
    for punto, conteggio in conteggio_punti.items():
       if conteggio > 10 and punto  not in punti_filtrati:
            punti_filtrati.append(punto)
            
    return punti_filtrati
 
#media posizioni

def filtra_punti(points, errore):
    x1, y1 = points
    
    punto_simile = False

    for x2, y2 in punti_filtrati:
        if abs(x1 - x2) < errore and abs(y1 - y2) < errore:
            punto_simile = True
            break 
    if not punto_simile:
            lista_conteggio_punti(points)
    






def publish_circle(punti_circo):

    pub1 = rospy.Publisher('/circle_topic', PointCloud2, queue_size=10)
    header1 = std_msgs.msg.Header()
    header1.stamp = rospy.Time.now()
    header1.frame_id = "odom"   
       
    for i,e in enumerate(punti_circo):
        points2 = [(xc,yc,0.0) for xc,yc in e]
        cloud_msg2 = pc2.create_cloud_xyz32(header1, points2)

    rate1 = rospy.Rate(1) 
    rate1.sleep()
    pub1.publish(cloud_msg2)
    points2.clear()






def circonferenza(centro, r, numero_punti=360):
    punti_circonferenza = []
    
    for i in range(numero_punti):
        angolo = 2 * math.pi * i / numero_punti
        x = centro[0] + r * math.cos(angolo)
        y = centro[1] + r * math.sin(angolo)
        punti_circonferenza.append([x, y])
    return punti_circonferenza


def publish_cylinders(points):
    
    
    marker_publisher = rospy.Publisher('/cylinder', MarkerArray, queue_size=10)
    rate = rospy.Rate(1) 
    cylinderarray = MarkerArray()
    for i,point in enumerate(points):
        
        cylinder = Marker()
        cylinder.id = i
        cylinder.header.frame_id = "base_link"
        cylinder.type = Marker.CYLINDER
        cylinder.pose.position = Point(point[0], point[1], 0.5)  
        cylinder.pose.orientation.w = 1.0 
        cylinder.scale.x = 1  # Diametro del cilindro
        cylinder.scale.y = 1  # Diametro del cilindro
        cylinder.scale.z = 1.0  # Altezza del cilindro
        cylinder.color.a = 1.0
        cylinder.color.r = 1.0
        cylinder.color.g = 0.0
        cylinder.color.b = 0.0
        cylinderarray.markers.append(cylinder)
    marker_publisher.publish(cylinderarray)
    

        
        


def main ():
   
    rospy.init_node('lettura_topic2')
    rospy.Subscriber('/center_point',  Point, callback ) 
    rospy.spin()
    


if __name__ == '__main__':
    
     
    main()
    

    
