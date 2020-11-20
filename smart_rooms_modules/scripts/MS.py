#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import sys
import signal

GPIO.setmode(GPIO.BCM)

DOOR_SENSOR_PIN = 18

RED_LIGHT = 9

isOpen = None
oldIsOpen = None

GPIO.setup(DOOR_SENSOR_PIN, GPIO.IN)

GPIO.setup(RED_LIGHT, GPIO.OUT)

while True:
    oldIsOpen = isOpen
    isOpen = GPIO.input(DOOR_SENSOR_PIN)

    if (isOpen and (isOpen != oldIsOpen)):
        print ("Space is unoccupied")
        GPIO.output(RED_LIGHT, True)
    elif (isOpen != oldIsOpen):
        print ("Space is occupied")
        GPIO.output(RED_LIGHT, False)

time.sleep(0.1)
