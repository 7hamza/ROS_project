#!/usr/bin/env python
#Importation de la bibliotheque Serial
import serial
from time import sleep

import rospy
from std_msgs.msg import String

def talker():
	#Ouverture Du Port Série, On met le XStick en dernier afin de lui forcer le /dev/ttyUSB1 
	port = "/dev/ttyUSB1"

	#Recuperation d'un handle pour la communication serie
	ser = serial.Serial(port, 9600, timeout=0)


	#Recuperation d'handle pour la publication sur le topic de presence
	pub = rospy.Publisher('/presence', String, queue_size=10)

	#Init du ROS NODE
	rospy.init_node('xstick_talker', anonymous=True)
	
	#Setting de la frequence de publication
	rate = rospy.Rate(10) # 10hz

	#Boucle de lecture sur le port serie puis publication sur le Topic 
	while not rospy.is_shutdown():
		#Lecture d'un caracter: Le code de presence
		data = ser.read(1)
		if len(data) > 0:
			print data
		sleep(0.5)
        pub_str = data 
        rospy.loginfo(pub_str)
        pub.publish(pub_str)
        rate.sleep()
	
	ser.close()

#Fonction Main
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass