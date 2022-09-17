# Laboratorio 2 - Robótica de Desarrollo, Intro a ROS 
Equipo de trabajo: 
- Luis Alberto Chavez Castro
- Camilo Pineda Correa

## Turtlesim





## Matlab



## Python
Se pide escribir un código que permita operar una tortuga del paquete turtlesim con el uso del teclado, cumpliendo con las siguientes especificaciones:

* Se debe mover hacia adelante y hacia atrás con las teclas W y S
* Debe girar en sentido horario y antihorario con las teclas D y A.
* Debe retornar a su posición y orientación centrales con la tecla R
* Debe dar un giro de 180° con la tecla ESPACIO

Para esto se accede al paquete hello_turtle de ROS que se encuentra en el workspace de Catkin generado, y dentro de la carpeta de scripts se genera uno nuevo con el nombre MyTeleopKey.py, dentro se genera el código correspondiente. Se utilizo como base un codigo ya existente que permitia operar la tortuga a través de las flechas del teclado.

El código del script se realizo de la sigiuente forma:

### Librerias
``` python
#LIBRERIAS
import rospy
from geometry_msgs.msg import Twist                                                     #Publicacion de mensajes de desplazamiento
from turtlesim.srv import TeleportAbsolute, TeleportRelative                            #Teleport al origen
from std_srvs.srv import Empty                                                          #Servicio limpiar pantalla.
import termios, sys, os
from numpy import pi
TERMIOS = termios
```

### Funciones
#### Publicador
``` python
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
```

#### Servicio teleport absoluto
Utilizada para retornar la tortuga a su posición y orientación original
``` python
def servTpAbs(x, y, ang):                                                               #Función para el servicio de teleport absoluto
    rospy.wait_for_service('/turtle1/teleport_absolute')                                #Se mantiene en espera de solicitud del servicio
    try:
        serv = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)       #Definición de publicador (topic y message type)
        teleport = serv(x, y, ang)
    except rospy.ServiceException as e:
        print(str(e))
```

#### Servicio teleport relativo
Utilizada para girar 180° la tortuga
``` python
def servTpRelat(linear, angular):                                                       #Función para servicio de teleport relativo
    rospy.wait_for_service('/turtle1/teleport_relative')                                #Se mantiene en espera de solicitud del servicio
    try:
        serv = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)       #Definición de publicador (topic y message type)
        teleport = serv(linear, angular)                                                
    except rospy.ServiceException as e:
        print(str(e))
```

#### Identificación de tecla presionada
Identificar cual de las teclas es presionada para luego ser asignada la función correspondiente
``` python
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
```

#### Servicio para limpiar la pantalla
Utilizada para limpiar la pantalla cuando se retorna la tortuga
``` python
def servClear():                                                                        #Función para limpiar recorrido
    rospy.wait_for_service('/clear')                                                    #Espera al servicio clear
    try:
        serv = rospy.ServiceProxy('/clear', Empty)                                      #Llamada al servicio
        clear = serv()
    except rospy.ServiceException as e:
        print(str(e))
```

### Main
Se manetiene en un ciclo evaluando que tecla es presionada y ejecutando la función respectiva
``` python
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
```

Luego de generado el script, se guarda.
Es necesario modificar el archivo CMakeList.txt agregando el script de la misma forma en que se ven los demás, en la sección de catkin_install_python.

Luego se realiza la ejecución de este con los siguientes pasos:

1. Lanzar 3 terminales
2. En la primera se inicia el nodo maestro con el comando
```
roscore
```
3. En la segunda se ejecuta el node de turtlesim, es decir la tortuga, con el comando
```
rosrun turtlesim turtlesim_node
```
4. En el tercero accedemos a la dirección del workspace de catkin y ejecutamos el comando para hacer la build del paquete
```
cd catkin_ws
catkin build hello_turtle
```
5. Luego en este mismo ingresamos 
```
source devel/setup.bash
```
6. Finalmente ejecutamos el script con el comando
```
rosrun hello_turtle myTeleopKey.py
```
lo que deja la terminal en espera por el ingreso de teclas, permitiendonos operar como se solicita la tortuga a través de W, A, S, D, SPACE y R.



## Conclusiones
* Turtlesim es una herramienta de aprendizaje muy util y completa que permite acercarse de la mejor forma al entorno de ROS y entenderlo facilmente, trabajando con nodos, paquetes, topicos y servicios.

* Python es una herramienta que permite la generación de scripts para la operación y trabajo a través de ROS, servicios y topicos; siendo un lenguaje muy utilizado, facilita su implementación.

* 