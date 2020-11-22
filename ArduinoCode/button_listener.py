#!/usr/bin/python3

# Author: Theodor Giles
# Created: 8/6/20
# Last Edited 8/7/20
# Description:
# This program is dedicated to starting up the robosub through button and buzzer peripherals on
# the raspberry pi.
#
import RPi.GPIO as GPIO
import time
import START_SUB as Mission

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(15, GPIO.IN)
print("Press button to start...")

MissionAlive = True

print("Button pushed... Starting up...")
time.sleep(3)
for i in range(2):
    GPIO.output(23, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(23, GPIO.LOW)
    time.sleep(0.01)
print("Running mission...")
Mission.run()
print("Mission complete...")
print("Cleaning up...")
GPIO.cleanup()
