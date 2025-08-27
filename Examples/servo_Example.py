import sys
sys.path.append("/home/pi_02/LittlePro_dir")
import LittlePro as lp
import RPi.GPIO as GPIO

lp.board_init()

lp.servo.begin()

def capture_cube():
	lp.servo.move(0,85)

def release_cube():
	lp.servo.move(0,175)
	
def up_cube():
	i = 180
	while i >= 90:
		lp.servo.move(1, i)
		i-=1
		lp.time.sleep(0.008)

def down_cube():
	i = 90
	while i >= 180:
		lp.servo.move(1, i)
		i+=1
		lp.time.sleep(0.008)



