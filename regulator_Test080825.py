import sys
sys.path.append("/home/pi_02/LittlePro_dir")
import LittlePro as lp
import RPi.GPIO as GPIO

lp.board_init()

GPIO.setmode(GPIO.BCM)
GPIO.setup(0,GPIO.IN)

print('Waiting for start...')
while GPIO.input(0) == GPIO.HIGH:
	pass
	
lp.time.sleep(1)
print('Start!') 

lp.groupSyncWrite = lp.motor_init()
lp.torque(1)

err_old = 0

def regulator_PD(kp, kd, speed):
	global err_old
	
	SL, SR = lp.get_sensor_value('A1'), lp.get_sensor_value('A2')
	#print(SL,SR)
	
	err = SL - (SR)
	P = kp * err
	D = kd * (err - err_old)
	U = P + D
	
	VL = speed - U
	VR = speed + U
	
	err_old = err
	print(err_old)
	vl = [lp.DXL_LOBYTE(lp.DXL_LOWORD(int(VL))), lp.DXL_HIBYTE(lp.DXL_LOWORD(int(VL))), lp.DXL_LOBYTE(lp.DXL_HIWORD(int(VL))), lp.DXL_HIBYTE(lp.DXL_HIWORD(int(VL)))]
	vr = [lp.DXL_LOBYTE(lp.DXL_LOWORD(int(VR))), lp.DXL_HIBYTE(lp.DXL_LOWORD(int(VR))), lp.DXL_LOBYTE(lp.DXL_HIWORD(int(VR))), lp.DXL_HIBYTE(lp.DXL_HIWORD(int(VR)))]
	return vl,vr


def cross_X(speed,kp,kd,cross_black): #0.15 0.2

	SCL, SCR = 0, 0

	while SCL+SCR < cross_black:
		SCL, SCR = lp.get_sensor_value('A0'), lp.get_sensor_value('A3')
		#print(SCL, SCR)
		
		vl, vr = regulator_PD(kp, kd, speed)
		lp.motor_start(vl, vr)
	lp.motor_stop(0)

	lp.motor_cantimetr_move(150,10)

cross_X(100, 0.15,0.2, 1880)
