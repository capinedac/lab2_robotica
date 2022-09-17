#Script de python correspondiente a un nodo tipo Teleop_key

#Mueve la tortuga Turtlesim con las teclas

#Se debe mover hacia adelante y hacia atr´as con las teclas W y S
#Debe girar en sentido horario y antihorario con las teclas D y A.
#Debe retornar a su posici´on y orientaci´on centrales con la tecla R
#Debe dar un giro de 180° con la tecla ESPACIO

#LIBRERIAS
import rospy
from geometry_msgs.msg import Twist                                                     #Publicacion de mensajes de desplazamiento
from turtlesim.srv import TeleportAbsolute, TeleportRelative                            #Teleport al origen
from std_srvs.srv import Empty                                                          #Servicio limpiar pantalla.
import termios, sys, os
from numpy import pi
TERMIOS = termios

#FUNCIONES
def pubVel(vel_x, ang_z, t):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)                     #Definición de publicador (topic, message type y tamaño de cola)
    rospy.init_node('velPub', anonymous=False)                                          #Inicio del nodo velPub
    vel = Twist()
    vel.linear.x = vel_x                                                                #Velocidad lineal
    vel.angular.z = ang_z                                                               #Velocidad angular
    #rospy.loginfo(vel)
    endTime = rospy.Time.now() + rospy.Duration(t)                                      #Límite de tiempo de publicación
    while rospy.Time.now() < endTime:                                                   #Publicacion
        pub.publish(vel)

def servTpAbs(x, y, ang):                                                               #Función para el servicio de teleport absoluto
    rospy.wait_for_service('/turtle1/teleport_absolute')                                #Se mantiene en espera de solicitud del servicio
    try:
        serv = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)       #Definición de publicador (topic y message type)
        teleport = serv(x, y, ang)
    except rospy.ServiceException as e:
        print(str(e))

def servTpRelat(linear, angular):                                                       #Función para servicio de teleport relativo
    rospy.wait_for_service('/turtle1/teleport_relative')                                #Se mantiene en espera de solicitud del servicio
    try:
        serv = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)       #Definición de publicador (topic y message type)
        teleport = serv(linear, angular)                                                
    except rospy.ServiceException as e:
        print(str(e))

def getkey():                                                                           #Función para identificar tecla presionada.
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c

def servClear():                                                                        #Función para limpiar recorrido
    rospy.wait_for_service('/clear')                                                    #Espera al servicio clear
    try:
        serv = rospy.ServiceProxy('/clear', Empty)                                      #Llamada al servicio
        clear = serv()
    except rospy.ServiceException as e:
        print(str(e))


#MAIN
if __name__ == '__main__':
    while 1: 
        key = getkey()
        if key == b'w':
            pubVel(1,0,0.1)                                                             #Desplazamiento lineal de frente por 0.1 seg
        if key == b's':
            pubVel(-1,0,0.1)                                                            #Desplazamiento lineal atrás por 0.1 seg
        if key == b'a':
            pubVel(0,1,0.1)                                                             #Rotación a izquierda por 0.1 seg
        if key == b'd':
            pubVel(0,-1,0.1)                                                            #Rotación a derecha por 0.1 seg
        if key == b' ':                                                                 
            servTpRelat(0,pi)                                                           #Rotación de 180°
        if key == b'r': 
            servTpAbs(5.54,5.54,0)                                                      #Teleport de la tortuga al centro
            servClear()                                                                 #Limpia recorrido de la pantalla