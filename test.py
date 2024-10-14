import os
os.environ['GPIOZERO_PIN_FACTORY'] = os.environ.get('GPIOZERO_PIN_FACTORY', 'mock')
from gpiozero import OutputDevice
import time

from picarx import Picarx
fc = Picarx()
angle = 35
#distance = fc.get_distance_at(angle)
while True:
            distance = fc.ultrasonic.read()
            print("distance: ",distance)
#print(f"Angle: {angle}, Distance: {distance} cm")

