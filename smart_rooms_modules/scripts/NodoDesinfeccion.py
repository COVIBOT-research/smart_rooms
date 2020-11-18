#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import sys
import signal
import rospy
from std_msgs.msg import String
from std_msgs.msg import boolenao	#importar dato de tipo booelano

############## Nodo Proceso de Desinfección ##############################
# Topico 'robot' (booleano) [Listen] que determina si el robot está o no en la habitación
# Topico 'break' (init) [Publish] envía advertencia acerca de irrupción en la sala durante el proceso de desinfección
# Topico 'firstroom' (String) [Publish] es la identificación de ese salon
# Nombre del nodo: desinfeccion
######################################################################

def callback(data):
     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
     pub = rospy.Publisher('break', String, queue_size=10)	#Topico: break
     salon = rospy.Publisher('firstroom', String, queue_size=10)	#Topico: FirstRoom (ID)
     rospy.init_node('desinfeccion', anonymous=True)	#Node_Name
     rate = rospy.Rate(10) # 10hz
     while not rospy.is_shutdown():
         hello_str = "Sistema OFF %s" % rospy.get_time()
         rospy.loginfo(hello_str)
         pub.publish(hello_str)
         rate.sleep()

def listener():
     rospy.Subscriber("robot", Bool, callback)	#Topico nodo orquestador: Robot
     rospy.spin()        

if __name__ == '__main__':
	 listener()
	 if listener() == True:	#Topico tipo boolenao: True (Informe presencia del robot en la sala)
	 	try:
     		GPIO.setmode(GPIO.BCM)

			DOOR_SENSOR_PIN = 10

			RED_LIGHT = 19

			isOpen = None
			oldIsOpen = None

			GPIO.setup(DOOR_SENSOR_PIN, GPIO.IN)

			GPIO.setup(RED_LIGHT, GPIO.OUT)

			while True:
    			oldIsOpen = isOpen
    			isOpen = GPIO.input(DOOR_SENSOR_PIN)

    			if (isOpen and (isOpen != oldIsOpen)):
        			print ("Espacio interrumpido")
        			talker()
        			GPIO.output(RED_LIGHT, True)
   				elif (isOpen != oldIsOpen):
        			print ("Espacio cerrado")
        			GPIO.output(RED_LIGHT, False)

			time.sleep(0.1)
         
     	except rospy.ROSInterruptException:
         	pass         