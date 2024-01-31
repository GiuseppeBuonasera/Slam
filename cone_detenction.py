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
from tf.transformations import euler_from_quaternion





punti_solidali =[]
posizione_macchina = []
ang = math.pi/720
xe = 0.32  #posizione del lidar rispetto alla macchina
yaw = 0




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
                ang1 = j * ang
                ang2 = math.pi - ang1
                y = lista[i]* (math.cos(ang2))   # 
                x = lista[i]* math.sin(ang2) + 0.32    #
                punti.append([x,y])
                i = j
            clusters.append(punti)
        else:
            i += 1
   
    for a,e in enumerate(clusters):
        if len(e)>10:
            xc, yc, r, sigma = taubinSVD(e)
            xa =xc*(math.cos(yaw))- yc*(math.sin(yaw))
            ya = yc*(math.cos(yaw))+ xc*(math.sin(yaw))
            coordinate_centri.append([xa,ya])
    punti_solidali=calcolo_punti_origine(coordinate_centri)
    print(punti_solidali)
    #print(coordinate_centri)

    
    
#funzione che calcola i centri dell'ostacolo rispetto all'origine    
def calcolo_punti_origine(coordinate_centri):
    
    punti_solidali.clear()
    
    a = posizione_macchina[0][0]  #coordinate lidar rispetto all'origine
    b = posizione_macchina[0][1] 
    #print(coordinate_centri)
    for e in coordinate_centri:
        
        xs =    a + e[0]
        ys =   b + e[1]
        punti_solidali.append([xs,ys])
        #print(e[0],e[1])
    publish_points(punti_solidali)
    posizione_macchina.clear()
    return punti_solidali
        

#funzione che ricava la posizione della macchina rispetto all'origine    
def posizione(dati):  
    global yaw
    posizione_macchina.clear()
    x = dati.pose.position.x
    y = dati.pose.position.y
    posizione_macchina.append([x,y])
    #print(posizione_macchina)
    orietamento = dati.pose.orientation
    lista_orientamento = [orietamento.x , orietamento.y, orietamento.z , orietamento.w]
    (roll, pitch, yaw) = euler_from_quaternion(lista_orientamento)
    #print(yaw)
    return posizione_macchina






#Funzione che pubblica i centri degli ostacoli
def publish_points(posizione_macchina):
    pub = rospy.Publisher('/center_point', Point, queue_size=10)
    rate = rospy.Rate(1000)

    if not rospy.is_shutdown():
        if posizione_macchina:
            for x, y in posizione_macchina:
                point_msg = Point()
                point_msg.x = x
                point_msg.y = y
                point_msg.z = 0.0
                pub.publish(point_msg)
        else:
            point_msg = Point()
            point_msg.x = 0.0
            point_msg.y = 0.0
            point_msg.z = 0.0
            pub.publish(point_msg)

    rate.sleep()






def main():
    rospy.init_node('lettura_topic')
    
    rospy.Subscriber('/steer_bot/message_to_tf/pose', PoseStamped, posizione)
    
    rospy.Subscriber('/rrbot/laser/scan', LaserScan, callback)
    
    rospy.spin()

    

if __name__ == '__main__':
    
    main()
