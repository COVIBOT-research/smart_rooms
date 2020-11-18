#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import sys
import signal
import rospy
import VL53L1X	#Library para sensores infrarrojos
from std_msgs.msg import String
from std_msgs.msg import init

############## Nodo Conteo de Personas ##############################
# Topico 'robot' (booleano) [Listen] que determina si el robot está o no en la habitación
# Topico 'nopersona' (init) [Publish] que arroja el valor de la cantidad de personas sensadad
# Topico 'firstroom' (String) [Publish] es la identificación de ese salon
# Topico 'exceder' (String)  [Publish] que advierte mediante el mensaje 'programar' que sobrepasó el... 
# ... limite de personas (20) y se debe trazar la ruta para desinfección
# Nombre del nodo: conteo
######################################################################

def callback(data):
     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
     pub = rospy.Publisher('nopersona', init, queue_size=10)	#Topico: nopersona (indica la cantidad de personas en la sala)
     salon = rospy.Publisher('firstroom', String, queue_size=10)	#Topico: FirstRoom (ID del salón)
     limite = rospy.Publisher('exceder', String, queue_size=10)	#Topico: exceder (Envía string al exceder la capacidad máxima)
     rospy.init_node('conteo', anonymous=True)	#Node_Name - Conteo de personas
     rate = rospy.Rate(10) # 10hz
     #while not rospy.is_shutdown():
         #hello_str = "Sistema OFF %s" % rospy.get_time()
         #rospy.loginfo(hello_str)
         #pub.publish(hello_str)
         #rate.sleep()

def listener():
     rospy.Subscriber("robot", Bool, callback)	#Topico nodo orquestador: Robot
     rospy.spin()

if __name__ == '__main__':
	 listener()
	 if listener() == False:	#Topico tipo boolenao: False (Informe NO presencia del robot en la sala)
	 	try:
	 		tof1 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
			tof1.open()
			tof1.start_ranging(2)	#Nivel medio de medición, rango de 0 a 3

			tof2 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x28)
			tof2.open()
			tof2.start_ranging(2)	#Nivel medio de medición, rango de 0 a 3

     		GPIO.setmode(GPIO.BCM)

			DOOR_SENSOR_PIN = 10

			RED_LIGHT = 19

			isOpen = None
			oldIsOpen = None
			d = None
			numero = 1

			GPIO.setup(DOOR_SENSOR_PIN, GPIO.IN)

			GPIO.setup(RED_LIGHT, GPIO.OUT)

			while True:
    			oldIsOpen = isOpen
    			isOpen = GPIO.input(DOOR_SENSOR_PIN)
    			distance1 = tof1.get_distance()	# en mm
    			distance2= tof2.get_distance()	# en mm

    			d = distance1 - distance2

    			if (d>0):
    				d = distance1 - distance2
    				if (d<0):
    					if (isOpen and (isOpen != oldIsOpen)):
    						numero = numero - 1
        					print ("Persona abandona la sala")
        					talker()
        					pub.publish(numero)
        					GPIO.output(RED_LIGHT, False)
   				if (isOpen and (isOpen != oldIsOpen)):
   					d = distance1 - distance2
   					if (d<0):
   						d = distance1 - distance2
   						if (d>0):
   							numero = numero + 1
   							print ("Persona ingresa a la sala")
        					talker()
        					pub.publish(numero)
        					if (numero > 20):
        						GPIO.output(RED_LIGHT, True)
   								print ("Exceso de personas en la sala")
   								talker()
   								mensaje = "Programar"	#Para programar desinfección
        						limite.publish(mensaje)
        					GPIO.output(RED_LIGHT, False)

				time.sleep(0.1)
         
     	except rospy.ROSInterruptException:
     		tof.stop_ranging()
     		pass

###################################