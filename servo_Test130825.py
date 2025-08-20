import sys
sys.path.append("/home/pi_02/LittlePro_dir")
import LittlePro as lp
import RPi.GPIO as GPIO

lp.board_init()

lp.servo.begin()

while 1:
	angle = lp.get_sensor_raw('A0')/22.75
	lp.servo.move(1, angle)
