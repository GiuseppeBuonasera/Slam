import rospy
import math
import numpy as np
import circle_fit as cf
from circle_fit import taubinSVD
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped , Point





punti_solidali =[]
posizione_macchina = []
ang = math.pi/720
xe = 0.3  #posizione del lidar rispetto alla macchina

def callback(data):

    
    data = data.ranges
    lista = list(data)
    clusters = []
    coordinate_centri = []

    
    i = 0
    #in questo ciclo vado a clusterizzare i punti ragruppandoli 
    while i < len(lista):
        if lista[i] != float('inf'):
            punti = []
            j = i 
            while j < len(lista) and lista[j] != float('inf'):
                j += 1
                ang1 = i * ang
                ang2 = math.pi - ang1
                x = (lista[i] * math.sin(ang2)) + xe 
                y = lista[i] * math.cos(ang2) 
                punti.append([x,y])
                i = j
            clusters.append(punti)
        else:
            i += 1
    
    for a,e in enumerate(clusters):
        xc, yc, r, sigma = taubinSVD(e)
        print(sigma)
        coordinate_centri.append([xc,yc])
        
        
    #con questa vado anche a pubblicare i punti
    punti_solidali= calcolo_punti_origine(coordinate_centri)
    

    print('POSIZIONE DEI PUNTI RIPETTO ALL ORIGINE: \n')
    print(punti_solidali)
    punti_solidali.clear()
    coordinate_centri.clear()
    
    clusters.clear()
    lista.clear()
    
    
#funzione che calcola i centri dell'ostacolo rispetto all'origine    
def calcolo_punti_origine(coordinate_centri):
    punti_solidali.clear()
    
    xm = posizione_macchina[0][1] 
    ym = posizione_macchina[0][0] 
    for e in coordinate_centri:
        xs = e[0] + xm 
        ys = e[1] + ym 
        punti_solidali.append([xs,ys])
    publish_points(punti_solidali)
    return punti_solidali
        

#funzione che ricava la posizione della macchina rispetto all'origine    
def posizione(dati):  
   
    posizione_macchina.clear()
    y = dati.pose.position.x
    x = dati.pose.position.y
    posizione_macchina.append([x,y])
    return posizione_macchina


#Funzione che pubblica i centri degli ostacoli
def publish_points(posizione_macchina):
    if not rospy.is_shutdown():
        pub = rospy.Publisher('/center_point', Point, queue_size=1000)
        rate = rospy.Rate(1000)  

        for x,y in posizione_macchina:
            point_msg = Point()
            point_msg.x = x  
            point_msg.y = y  
            point_msg.z = 0.0  

            pub.publish(point_msg)
    else:
        pub = rospy.Publisher('/center_point', Point, queue_size=1000)
        rate = rospy.Rate(1000)
        for x,y in posizione_macchina:
            point_msg = Point()
            point_msg.x = 0.0
            point_msg.y = 0.0 
            point_msg.z = 0.0 
    rate.sleep()





def main():
    rospy.init_node('lettura_topic')
    
    rospy.Subscriber('/steer_bot/message_to_tf/pose', PoseStamped, posizione)
    
    rospy.Subscriber('/rrbot/laser/scan', LaserScan, callback)
    
    rospy.spin()

    

if __name__ == '__main__':
    
    main()