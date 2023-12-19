
import rospy
import math
from geometry_msgs.msg import PoseStamped , Point
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

errore = 0.5
punti_filtrati1 =[]
punti_filtrati = []


def callback(data):
    punti_filtrati1 = []
    punti_circo = []
    point=[]
    x = data.x
    y = data.y
    point.append([x,y])
    
    filtra_punti(point,errore)
    conteggio_punti(punti_filtrati1)
    for a in punti_filtrati:
        
        punti_cerchi = circonferenza(a, r=0.5, numero_punti=360)
        punti_circo.append(punti_cerchi)
        publish_circle(punti_circo)
    
    point.clear()
    punti_circo.clear() 



def conteggio_punti(punti):
    punti_unici = []
    conteggio_punti = {}
    punti_filtrati = []

    for x, y in punti:
        # Condizione basata sulla differenza tra x e 0.03, e y e 0.03
        condizione = [abs(x - 0.03), abs(y - 0.03)]

        if tuple(condizione) not in conteggio_punti:
            conteggio_punti[tuple(condizione)] = 1
            punti_unici.append(tuple(condizione))
        else:
            conteggio_punti[tuple(condizione)] += 1

    for punto, conteggio in conteggio_punti.items():
        if conteggio > 10:
            punti_filtrati.append(punto)

    print("Punti filtrati:", punti_filtrati)
    return punti_filtrati


  








def filtra_punti(points, errore):
    for valori in points:
        x1, y1 = valori
        punto_simile = False

        for x2, y2 in punti_filtrati1:
            if abs(x1 - x2) < errore and abs(y1 - y2) < errore:
                punto_simile = True
                break 
        if not punto_simile:
            punti_filtrati1.append([x1, y1])
   
    return punti_filtrati1






























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
    