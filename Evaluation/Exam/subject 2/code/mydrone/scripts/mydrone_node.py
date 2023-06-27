#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# For obfuscation, see https://pyob.oxyry.com/ ;-)

import rospy
import numpy as np
from std_msgs.msg import Float32
from mydrone.srv import alt_warning, alt_warningResponse

# ====================
# Class describing the drone
# ====================
class mydrone:

    def __init__(self):

        print("Launching mydrone:")
        self.rate_pressure=1.33    # Publication rate for pressure
        self.rate_temp=0.777       # Publication rate for temperature
        self.freq_simu = 1/100     # Data are simulated with this frequency

        # Simulated data init
        self.temperature = 0
        self.pressure = 0
        self.altitude = 0

        # Definition of the messages to publish
        self.message_pressure=Float32()
        self.message_temp=Float32()

        # Init everything (node + service)
        rospy.init_node('mydrone', anonymous=False)
        self.time_init=rospy.get_time()
        rospy.Service('altitude_warning', alt_warning, self.led_alt_warning)
        self.pub_pressure = rospy.Publisher('/pressure', Float32, queue_size=10)
        self.pub_temp = rospy.Publisher('/temperature', Float32, queue_size=10)

        # Define publication rates
        rospy.Timer(rospy.Duration(1.0/self.rate_pressure), self.publish_pressure)
        print(" -> Publishing pressure data to /pressure.")
        rospy.Timer(rospy.Duration(1.0/self.rate_temp), self.publish_temp)
        print(" -> Publishing temperature data to /temperature.")


    # ---------------------------
    # Publisher of the temperature data
    # ---------------------------
    def publish_temp(self, event):
        
        temp_min = 0
        temp_max = 22
        temp_mean = (temp_max+temp_min)/2
        # Simulate pressure values with a small noise
        self.temperature = temp_mean + \
                      (temp_max-temp_mean)*np.cos(2*np.pi*self.freq_simu*(rospy.get_time()-self.time_init)) + \
                      ((np.random.random_sample(size=1)-0.5)*2)*0.5
        # Create the message and publish
        self.message_temp.data = self.temperature
        self.pub_temp.publish(self.message_temp) 


    # ---------------------------
    # Publisher of the pressure data
    # ---------------------------
    def publish_pressure(self,event):

        alt_min = 100
        alt_max = 1500
        alt_mean = (alt_min+alt_max)/2
        # Simulate altitude values
        self.altitude = alt_mean - \
                        (alt_max-alt_mean)*np.cos(2*np.pi*self.freq_simu*(rospy.get_time()-self.time_init))

        # Compute corresponding pressure values
        # See https://www.123calculus.com/calculer-pression-altitude-page-8-30-300.html
        P0 = 101325                   # Pression au niveau de la mer
        mu = 0.0289644                # Masse molaire moyenne de l'air
        g = 9.80665                   # Pesanteur terrestre
        R = 8.31432                   # Constante des gaz parfaits
        T = self.temperature + 273.15 # Temperature
        self.pressure = P0 * np.exp(-mu*g*self.altitude/(R*T)) + \
                      ((np.random.random_sample(size=1)-0.5)*2)*500

        # Create the message and publish
        self.message_pressure.data = self.pressure
        self.pub_pressure.publish(self.message_pressure) 

    
    # ---------------------------
    # Service
    # ---------------------------
    def led_alt_warning(self, request):

        # If we want to put LED in RED
        if request.message == "WARNING":
            response = "RED"
            rospy.loginfo("LED set to RED on the remote.")
        # Else in GREEN
        elif request.message == "NORMAL":
            response = "GREEN"
            rospy.loginfo("LED set to GREEN on the remote.")
        # Else ERROR
        else:
            rospy.loginfo("Bad LED service request.")
            response = "ERROR"

        return alt_warningResponse(response)


# ====================
# Main function
# ====================
if __name__ == '__main__':
    try:
        robot = mydrone()
        print("Running...")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
