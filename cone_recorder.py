import rospy
import math
from geometry_msgs.msg import PoseStamped , Point
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2 as pc2
import threading
import time
import std_msgs.msg

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
        
        buffer.clear()

        count = 0
    else:
        count +=1
    
   
    #pubblicazione cerchi su rviz
    for a in punti_filtrati:
        punti_cerchi = circonferenza(a, r=0.5, numero_punti=360)
        punti_circo.append(punti_cerchi)
        publish_circle(punti_circo)

    print(punti_filtrati)
    punti_circo.clear()



def lista_conteggio_punti(punti):
    x,y = punti
    
    trovato = False
    for (x1, y1), conteggio in conteggio_punti.items():
        differenza_x = abs(x - x1)
        differenza_y = abs(y - y1)
        condizione = differenza_x <= 0.5 and differenza_y <= 0.5
        if condizione:
            conteggio_punti[x1,y1] = conteggio_punti.get((x1,y1), 0) + 1
            trovato = True
            break
    if not trovato:
        conteggio_punti[(x, y)] = 1

    
    for punto, conteggio in conteggio_punti.items():
       if conteggio > 10 and punto  not in punti_filtrati:
            punti_filtrati.append(punto)
            
    return punti_filtrati
 


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

    pub1 = rospy.Publisher('/circle_topic', PointCloud2, queue_size=100)
    header1 = std_msgs.msg.Header()
    header1.stamp = rospy.Time.now()
    header1.frame_id = "odom"   
       
    for i,e in enumerate(punti_circo):
        points2 = [(xc,yc,0.0) for xc,yc in e]
        cloud_msg2 = pc2.create_cloud_xyz32(header1, points2)

    rate1 = rospy.Rate(2000) 
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




def main ():
   
    rospy.init_node('lettura_topic2')
    rospy.Subscriber('/center_point',  Point, callback ) 
    rospy.spin()
    


if __name__ == '__main__':
    
     
    main()
    

    
