#!/usr/bin/env python
from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

buzzTime = 0.8
buzzDelay = 2

buzzerPin = 4

GPIO.setup(buzzerPin, GPIO.OUT)

print("buzz")
GPIO.output(buzzerPin, True)
sleep(buzzTime)
GPIO.output(buzzerPin, False)
sleep(buzzDelay)
