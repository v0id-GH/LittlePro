import os, math, time, adafruit_tcs34725, smbus
import board as bd
from dynamixel_sdk import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

motors_ID = [0,1] #0 - left, 1 - right
sensor_max_value = 1000 #1000 - white | 100-300 - black

#for direct turn:
diametr = 5.2
center_distance = 15
#----------------
max_value = 1000

PROTOCOL_VERSION = 2.0
DEVICENAME = '/dev/ttyUSB0' #connect driver to rpi (micro-usb)
BAUDRATE = 57600

#adresses:
torqueAddr = 64 
presentPositionAddr = 132
#goalVelocityAddr = 104
#goalPositionAddr = 116

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

from DF import DFRobot_Expansion_Board_IIC as Board
from DF import DFRobot_Expansion_Board_Servo as Servo

board = Board(1, 0x10)    # Select i2c bus 1, set address to 0x10
servo = Servo(board)

def board_detect():
  l = board.detecte()
  #print("Board list conform:")
  #print(l)

def print_board_status():
  if board.last_operate_status == board.STA_OK:
    print("board status: everything ok")
  elif board.last_operate_status == board.STA_ERR:
    print("board status: unexpected error")
  elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
    print("board status: device not detected")
  elif board.last_operate_status == board.STA_ERR_PARAMETER:
    print("board status: parameter error")
  elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
    print("board status: unsupport board framware version")

def board_init():
	board_detect()    # If you forget address you had set, use this to detected them, must have class instance

	while board.begin() != board.STA_OK:    # Board begin and check board status
		#print_board_status()
		print("board begin faild")
		time.sleep(0.5)
	#print("board begin success")

	board.set_adc_enable()
#-----------------------------------------------------------------------
#base functions:

def open_port():
	if portHandler.openPort():
		pass
		#print("Succeeded to open the port")
	else:
		#print("Failed to open the port")
		print("Press any key to terminate...")
		getch()
		quit()

#to kirill: try call this once at beginnig
def set_baudrate():
	if portHandler.setBaudRate(BAUDRATE):
		pass
		#print("Succeeded to change the baudrate")
	else:
		print("Failed to change the baudrate")
		print("Press any key to terminate...")
		getch()
		quit()

def close_port():
	portHandler.closePort()
	#print('Succeeded to close the port')
	
	'''
	if portHandler.closePort():
		pass
		print("Succeeded to close the port")
	else:
		print("Failed to close the port")
		print("Press any key to terminate...")
		getch()
		quit()
	'''

def set_motor_direction(direction):
	if direction == 1:
		#print('fwd')
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[0], 10, 1)
		#if dxl_comm_result != COMM_SUCCESS:
			#print('comm err')
		#elif dxl_error != 0:
			#print('err')
		#else: print('motor1: set direction')
		
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[1], 10, 0)
		#if dxl_comm_result != COMM_SUCCESS:
			#print('comm err')
		#elif dxl_error != 0:
			#print('err')
		#else: print('motor2: set direction')
		
	elif direction == -1:
		#print('bwd')
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[0], 10, 0)
		#if dxl_comm_result != COMM_SUCCESS:
			#print('comm err')
		#elif dxl_error != 0:
			#print('err')
		#else: print('motor1: set direction')
		
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[1], 10, 1)
		#if dxl_comm_result != COMM_SUCCESS:
			#print('comm err')
		#elif dxl_error != 0:
			#print('err')
		#else: print('motor2: set direction')
	
	else:
		print('UNKNW-DIR error')
		quit()
		
def set_motor_turn(turn):
	if turn == 1:
		print('right')
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[0], 10, 1)
		if dxl_comm_result != COMM_SUCCESS:
			print('comm err')
		elif dxl_error != 0:
			print('err')
		#else: print('motor1: set direction')
		
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[1], 10, 1)
		if dxl_comm_result != COMM_SUCCESS:
			print('comm err')
		elif dxl_error != 0:
			print('err')
		#else: print('motor2: set direction')
		
	elif turn == -1:
		print('left')
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[0], 10, 0)
		if dxl_comm_result != COMM_SUCCESS:
			print('comm err')
		elif dxl_error != 0:
			print('err')
		#else: print('motor1: set direction')
		
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[1], 10, 0)
		if dxl_comm_result != COMM_SUCCESS:
			print('comm err')
		elif dxl_error != 0:
			print('err')
		#else: print('motor2: set direction')
	
	else:
		print('UNKNW-TRN error')
		quit()

def clear_motor_position():
	dxl_comm_result, dxl_error = packetHandler.clearMultiTurn(portHandler, motors_ID[0])
	#if dxl_comm_result != COMM_SUCCESS:
		#print('comm err')
	#elif dxl_error != 0:
		#print('err')
	#else: print('motor1: cleared')
	cleared_pos1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, motors_ID[0], presentPositionAddr)
	
	dxl_comm_result, dxl_error = packetHandler.clearMultiTurn(portHandler, motors_ID[1])
	#if dxl_comm_result != COMM_SUCCESS:
		#print('comm err')
	#elif dxl_error != 0:
		#print('err')
	#else: print('motor2: cleared')
	cleared_pos2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, motors_ID[1], presentPositionAddr)
	
	return cleared_pos1, cleared_pos2
	
def torque(state):
	if state:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[0], torqueAddr, state)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		#else: print('mototr1: torque on')
				
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[1], torqueAddr, state)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		#else: print('mototr2: torque on')
	else:
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[0], torqueAddr, state)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		#else: print('mototr1: torque off')
				
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[1], torqueAddr, state)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		#else: print('mototr2: torque off')
	
def set_motor_mode(mode):
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[0], 11, mode)
	#if dxl_comm_result != COMM_SUCCESS:
		#print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	#elif dxl_error != 0:
		#print("%s" % packetHandler.getRxPacketError(dxl_error))
	#else:
		#print('motor1: mode selected')
		
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motors_ID[1], 11, mode)	
	#if dxl_comm_result != COMM_SUCCESS:
		#print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	#elif dxl_error != 0:
		#print("%s" % packetHandler.getRxPacketError(dxl_error))
	#else:
		#print('motor2: mode selected')

#-----------------------------------------------------------------------
#complex functions:

def motor_position_now():
	groupSyncRead = GroupSyncRead(portHandler, packetHandler, 132, 4) # 132-position 128-velocity
	
	# Syncread present position
	dxl_comm_result = groupSyncRead.txRxPacket()
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

      # Check if groupsyncread data of Dynamixel#1 is available
	dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[0], presentPositionAddr, 4)
	if dxl_getdata_result != True:
		print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[0])
		quit()

      # Check if groupsyncread data of Dynamixel#2 is available
	dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[1], presentPositionAddr, 4)
	if dxl_getdata_result != True:
		print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[1])
		quit()

       # Get Dynamixel#1 present position value
	position1_now = groupSyncRead.getData(motors_ID[0], presentPositionAddr, 4)
       
       # Get Dynamixel#2 present position value
	position2_now = groupSyncRead.getData(motors_ID[1], presentPositionAddr, 4)
		
	return position1_now, position2_now, 

def motor_init():
	set_baudrate()

	open_port()
	
	set_motor_mode(1)
	
	set_motor_direction(1)
	
	groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 104, 4) #104 - speed address
	
	torque(1)
	return groupSyncWrite

def motor_start(parametr_speed1, parametr_speed2):
	
	# Add M1 goal speed value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(motors_ID[0], parametr_speed1)
	if dxl_addparam_result != True:
		print("[ID:%03d] groupSyncWrite addparam failed" % motors_ID[0])
		quit()

	# Add M2 goal speed value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(motors_ID[1], parametr_speed2)
	if dxl_addparam_result != True:
		print("[ID:%03d] groupSyncWrite addparam failed" % motors_ID[1])
		quit()

	# Syncwrite goal speed
	dxl_comm_result = groupSyncWrite.txPacket()
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
			
	# Clear syncwrite parameter storage
	groupSyncWrite.clearParam()

def motor_stop(force_stop):
	if force_stop:
		#print('force stop: on')
		groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 104, 4) #104 - speed address
		
		#parametrs for Syncwrite 
		parametr_speed1 = [DXL_LOBYTE(DXL_LOWORD(0)), DXL_HIBYTE(DXL_LOWORD(0)), DXL_LOBYTE(DXL_HIWORD(0)), DXL_HIBYTE(DXL_HIWORD(0))]
		parametr_speed2 = [DXL_LOBYTE(DXL_LOWORD(0)), DXL_HIBYTE(DXL_LOWORD(0)), DXL_LOBYTE(DXL_HIWORD(0)), DXL_HIBYTE(DXL_HIWORD(0))]
		
		# Add M1 goal speed value to the Syncwrite parameter storage
		dxl_addparam_result = groupSyncWrite.addParam(motors_ID[0], parametr_speed1)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncWrite addparam failed" % motors_ID[0])
			quit()

		# Add M2 goal speed value to the Syncwrite parameter storage
		dxl_addparam_result = groupSyncWrite.addParam(motors_ID[1], parametr_speed2)
		if dxl_addparam_result != True:
			print("[ID:%03d] groupSyncWrite addparam failed" % motors_ID[1])
			quit()

		# Syncwrite goal speed
		dxl_comm_result = groupSyncWrite.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
				
		# Clear syncwrite parameter storage
		groupSyncWrite.clearParam()
		
		time.sleep(0.3)
		
		torque(0)
		
		close_port()
	else:
		#print('force stop: off')
		torque(0)
		
		close_port()

def motor_set_velocity(speed):
	# Set motor velocity --------------------------------------------------------
	dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motors_ID[0], 112, speed)
	
	if dxl_comm_result != COMM_SUCCESS:
	    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
	    print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
	    print('motor1: velocity sets')
	 
	dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motors_ID[1], 112, speed)
	
	if dxl_comm_result != COMM_SUCCESS:
	    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
	    print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
	    print('motor2: velocity sets')
	
def motor_position_move(speed, position):
	
	set_baudrate()

	open_port()
	
	set_motor_mode(4)
	
	clear_motor_position()
	
	if speed > 0:
		set_motor_direction(1)
	elif speed < 0:
		set_motor_direction(-1)
	else:
		print('SP0 error')
		quit()
		
	groupSyncRead = GroupSyncRead(portHandler, packetHandler, 132, 4) # 132-position 128-velocity
	
	# Add Syncread param  
	dxl_addparam_result = groupSyncRead.addParam(motors_ID[0])
	if dxl_addparam_result != True:
	    print("[ID:%03d] groupSyncRead addparam failed" % motors_ID[0])
	    quit()

	dxl_addparam_result = groupSyncRead.addParam(motors_ID[1])
	if dxl_addparam_result != True:
	    print("[ID:%03d] groupSyncRead addparam failed" % motors_ID[1])
	    quit()
	
	# Syncread present position
	dxl_comm_result = groupSyncRead.txRxPacket()
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

	# Check if groupsyncread data of Dynamixel#1 is available
	dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[0], presentPositionAddr, 4)
	if dxl_getdata_result != True:
		print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[0])
		quit()

	# Check if groupsyncread data of Dynamixel#2 is available
	dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[1], presentPositionAddr, 4)
	if dxl_getdata_result != True:
		print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[1])
		quit()

	# Get Dynamixel#1 present position value
	position1_now = groupSyncRead.getData(motors_ID[0], presentPositionAddr, 4)

	# Get Dynamixel#2 present position value
	position2_now = groupSyncRead.getData(motors_ID[1], presentPositionAddr, 4)
	
	print('position M1 now: ' + str(position1_now))
	
	groupSyncRead.clearParam()
	
	# Add Syncread param  
	dxl_addparam_result = groupSyncRead.addParam(motors_ID[0])
	if dxl_addparam_result != True:
	    print("[ID:%03d] groupSyncRead addparam failed" % motors_ID[0])
	    quit()

	dxl_addparam_result = groupSyncRead.addParam(motors_ID[1])
	if dxl_addparam_result != True:
	    print("[ID:%03d] groupSyncRead addparam failed" % motors_ID[1])
	    quit()
	    
	groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 116, 4) #104 - speed address
	
	motor_set_velocity(abs(speed))

	torque(1)
	
	print('goal position M1: ' + str(position1_now+position))
	#parametrs for Syncwrite 
	parametr_position1 = [DXL_LOBYTE(DXL_LOWORD(position1_now+position+50)), DXL_HIBYTE(DXL_LOWORD(position1_now+position+50)), DXL_LOBYTE(DXL_HIWORD(position1_now+position+50)), DXL_HIBYTE(DXL_HIWORD(position1_now+position+50))]
	parametr_position2 = [DXL_LOBYTE(DXL_LOWORD(position2_now+position+50)), DXL_HIBYTE(DXL_LOWORD(position2_now+position+50)), DXL_LOBYTE(DXL_HIWORD(position2_now+position+50)), DXL_HIBYTE(DXL_HIWORD(position2_now+position+50))]
	
	dxl_addparam_result = groupSyncWrite.addParam(motors_ID[0], parametr_position1)
	if dxl_addparam_result != True:
		print("[ID:%03d] groupSyncWrite addparam failed" % motors_ID[0])
		quit()

	# Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(motors_ID[1], parametr_position2)
	if dxl_addparam_result != True:
		print("[ID:%03d] groupSyncWrite addparam failed" % motors_ID[1])
		quit()

	# Syncwrite goal position
	dxl_comm_result = groupSyncWrite.txPacket()
	if dxl_comm_result != COMM_SUCCESS:
	 print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

	# Clear syncwrite parameter storage
	groupSyncWrite.clearParam()
	
	position1_rightnow_old = 0
	while 1:
		
		# Syncread present position
		dxl_comm_result = groupSyncRead.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

		# Check if groupsyncread data of Dynamixel#1 is available
		dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[0], presentPositionAddr, 4)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[0])
			quit()

		# Check if groupsyncread data of Dynamixel#2 is available
		dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[1], presentPositionAddr, 4)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[1])
			quit()

		# Get Dynamixel#1 present position value
		position1_rightnow = groupSyncRead.getData(motors_ID[0], presentPositionAddr, 4)

		# Get Dynamixel#2 present position value
		position2_rightnow = groupSyncRead.getData(motors_ID[1], presentPositionAddr, 4)
		print('position M1 rightnow: ' + str(position1_rightnow))
		
		time.sleep(0.04)
		
		if position1_rightnow == position1_rightnow_old:
			print('position M1 is done')
			break
		
		position1_rightnow_old = position1_rightnow
			
	torque(0)
	
	close_port()

def motor_cantimetr_move(speed, cm):
	lenght_of_circle=2*3.14*(5.2/2)
	#print(lenght_of_circle)
	one_cm = 4096 / lenght_of_circle
	pos = one_cm * cm
	motor_position_move(speed, int(pos))


	

def give_degrees_turn(degrees):
	#attitude
	a = 360 / degrees
	
	#circumference of wheel
	c = 2 * 3.14 * (diametr/2)
	print('c =' + str(c))
	
	#cm in one pos
	temp = c/4096
	print('temp =' + str(temp))
	
	#path
	s = (2 * 3.14 * (center_distance/2)) / a
	print('s =' + str(s))
	
	given_position = int(s/temp)
	print('given_position =' + str(given_position))
	
	return given_position

def motor_position_turn(speed, position):
	
	set_baudrate()

	open_port()
	
	set_motor_mode(4)
	
	clear_motor_position()
	
	if speed > 0:
		set_motor_turn(1)
	elif speed < 0:
		set_motor_turn(-1)
	else:
		print('SP0 error')
		quit()
		
	groupSyncRead = GroupSyncRead(portHandler, packetHandler, 132, 4) # 132-position 128-velocity
	
	# Add Syncread param  
	dxl_addparam_result = groupSyncRead.addParam(motors_ID[0])
	if dxl_addparam_result != True:
	    print("[ID:%03d] groupSyncRead addparam failed" % motors_ID[0])
	    quit()

	dxl_addparam_result = groupSyncRead.addParam(motors_ID[1])
	if dxl_addparam_result != True:
	    print("[ID:%03d] groupSyncRead addparam failed" % motors_ID[1])
	    quit()
	
	# Syncread present position
	dxl_comm_result = groupSyncRead.txRxPacket()
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

	# Check if groupsyncread data of Dynamixel#1 is available
	dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[0], presentPositionAddr, 4)
	if dxl_getdata_result != True:
		print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[0])
		quit()

	# Check if groupsyncread data of Dynamixel#2 is available
	dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[1], presentPositionAddr, 4)
	if dxl_getdata_result != True:
		print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[1])
		quit()

	# Get Dynamixel#1 present position value
	position1_now = groupSyncRead.getData(motors_ID[0], presentPositionAddr, 4)

	# Get Dynamixel#2 present position value
	position2_now = groupSyncRead.getData(motors_ID[1], presentPositionAddr, 4)
	
	#print('position M1 now: ' + str(position1_now))
	
	groupSyncRead.clearParam()
	
	# Add Syncread param  
	dxl_addparam_result = groupSyncRead.addParam(motors_ID[0])
	if dxl_addparam_result != True:
	    print("[ID:%03d] groupSyncRead addparam failed" % motors_ID[0])
	    quit()

	dxl_addparam_result = groupSyncRead.addParam(motors_ID[1])
	if dxl_addparam_result != True:
	    print("[ID:%03d] groupSyncRead addparam failed" % motors_ID[1])
	    quit()
	    
	groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 116, 4) #104 - speed address
	
	motor_set_velocity(abs(speed))

	torque(1)
	
	#print('goal position M1: ' + str(position1_now+position))
	#parametrs for Syncwrite 
	parametr_position1 = [DXL_LOBYTE(DXL_LOWORD(position1_now+position+50)), DXL_HIBYTE(DXL_LOWORD(position1_now+position+50)), DXL_LOBYTE(DXL_HIWORD(position1_now+position+50)), DXL_HIBYTE(DXL_HIWORD(position1_now+position+50))]
	parametr_position2 = [DXL_LOBYTE(DXL_LOWORD(position2_now+position+50)), DXL_HIBYTE(DXL_LOWORD(position2_now+position+50)), DXL_LOBYTE(DXL_HIWORD(position2_now+position+50)), DXL_HIBYTE(DXL_HIWORD(position2_now+position+50))]
	
	dxl_addparam_result = groupSyncWrite.addParam(motors_ID[0], parametr_position1)
	if dxl_addparam_result != True:
		print("[ID:%03d] groupSyncWrite addparam failed" % motors_ID[0])
		quit()

	# Add Dynamixel#2 goal position value to the Syncwrite parameter storage
	dxl_addparam_result = groupSyncWrite.addParam(motors_ID[1], parametr_position2)
	if dxl_addparam_result != True:
		print("[ID:%03d] groupSyncWrite addparam failed" % motors_ID[1])
		quit()

	# Syncwrite goal position
	dxl_comm_result = groupSyncWrite.txPacket()
	if dxl_comm_result != COMM_SUCCESS:
	 print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

	# Clear syncwrite parameter storage
	groupSyncWrite.clearParam()
	
	position1_rightnow_old = 0
	while 1:
		
		#print(get_Compass_value())
		
		# Syncread present position
		dxl_comm_result = groupSyncRead.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

		# Check if groupsyncread data of Dynamixel#1 is available
		dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[0], presentPositionAddr, 4)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[0])
			quit()

		# Check if groupsyncread data of Dynamixel#2 is available
		dxl_getdata_result = groupSyncRead.isAvailable(motors_ID[1], presentPositionAddr, 4)
		if dxl_getdata_result != True:
			print("[ID:%03d] groupSyncRead getdata failed" % motors_ID[1])
			quit()

		# Get Dynamixel#1 present position value
		position1_rightnow = groupSyncRead.getData(motors_ID[0], presentPositionAddr, 4)

		# Get Dynamixel#2 present position value
		position2_rightnow = groupSyncRead.getData(motors_ID[1], presentPositionAddr, 4)
		#print('position M1 rightnow: ' + str(position1_rightnow))
		
		time.sleep(0.03)
		
		if position1_rightnow == position1_rightnow_old:
			print('position M1 is done')
			break
		
		position1_rightnow_old = position1_rightnow
			
	torque(0)
	
	close_port()

def motor_degrees_turn(speed, degrees):
	motor_position_turn(speed, give_degrees_turn(degrees)-55)

def get_sensor_raw(sensorPort):
	
	if sensorPort == 'A0':
		value = board.get_adc_value(board.A0)
		return value
	elif sensorPort == 'A1':
		value = board.get_adc_value(board.A1)
		return value
	elif sensorPort == 'A2':
		value = board.get_adc_value(board.A2)
		return value-65
	elif sensorPort == 'A3':
		value = board.get_adc_value(board.A3)
		return value
	else:
		print('no such port error')
		quit

def get_sensor_value(sensorPort):
	value = get_sensor_raw(sensorPort)
	if value >= max_value:
		value = max_value
		return value
	else:
		return value
		
#TCS34725:
#i2c = bd.I2C()
#sensor = adafruit_tcs34725.TCS34725(i2c)
#sensor.integration_time = 100  # In milliseconds

def get_rawRGB_value():
	r, g, b, c = sensor.color_raw
	#print(f"RGB: R={r}, G={g}, B={b}, Clear={c}")
	return r, g, b, c

def get_RGB_value():
	r, g, b, c = get_rawRGB_value()
	
	#Black
	if r < 100 and g < 100 and b < 100:
		r = 0
		g = 0
		b = 0
		color = 'Black'

		return r, g, b, color

	#White 
	elif r > 500 and g > 500 and b > 500:
		r = 255
		g = 255
		b = 255
		color = 'White'

		return r, g, b, color
		
	#Red
	elif r > b and r > g and c < 1000:
		r = 255
		g = 0
		b = 0
		color = 'Red'
		
		return r, g, b, color
	
	#Yellow
	elif r > b and r > g and c > 1000:
		r = 255
		g = 255
		b = 0
		color = 'Yellow'

		return r, g, b, color

	#Green
	elif g > r + b:
		g = 255
		r = 0
		b = 0
		color = 'Green'

		return r, g, b, color 

	#Blue
	elif b > g and b > r:
		b = 255
		r = 0
		g = 0
		color = 'Blue'

		return r, g, b, color
		
	else:
		print('Error')
		quit()

def get_HSV_value():
	r, g, b, color = get_RGB_value()

	r /= 255
	g /= 255
	b /= 255

	maximun = max(r,g,b)
	minimum = min(r,g,b)

	v = maximun * 100

	if minimum == maximun:
		return 0, 0, v

	diff = maximun - minimum

	if r == maximun:
		h = (g - b) / diff

	elif g == maximun:
		h = 2 + (b - r) / diff

	else:
		h = 4 + (r - g) / diff

	h *= 60 
	s = diff / maximun * 100

	return h, s, v

def get_Color_value():
	r, g, b, color = get_RGB_value()
	return color
	
#HMC5883L:
#ADDRESS = 0x1E
#CONFIG_A = 0x00
#CONFIG_B = 0x01
#MODE = 0x02
#X_MSB = 0x03
#Z_MSB = 0x05
#Y_MSB = 0x07
 
bus = smbus.SMBus(1)

def setup():
    bus.write_byte_data(ADDRESS, CONFIG_A, 0x70)  # Set to 8 samples @ 15Hz
    bus.write_byte_data(ADDRESS, CONFIG_B, 0x20)  # 1.3 gain LSb / Gauss 1090 (default)
    bus.write_byte_data(ADDRESS, MODE, 0x00)  # Continuous measurement mode
 
def read_raw_data(addr):
    # Read raw 16-bit value
    high = bus.read_byte_data(ADDRESS, addr)
    low = bus.read_byte_data(ADDRESS, addr+1)
    
    # Combine them to get a 16-bit value
    value = (high << 8) + low
    if value > 32768:  # Adjust for 2's complement
        value = value - 65536
    return value
 
def compute_heading(x, y):
    # Calculate heading in radians
    heading_rad = math.atan2(y, x)
    
    # Adjust for declination angle (e.g. 0.22 for ~13 degrees)
    declination_angle = 0.488
    heading_rad += declination_angle
    
    # Correct for when signs are reversed.
    if heading_rad < 0:
        heading_rad += 2 * math.pi
 
    # Check for wrap due to addition of declination.
    if heading_rad > 2 * math.pi:
        heading_rad -= 2 * math.pi
 
    # Convert radians to degrees for readability.
    heading_deg = heading_rad * (180.0 / math.pi)
    
    return heading_deg

def get_Compass_value():
    setup()
	
    x = read_raw_data(X_MSB)
    y = read_raw_data(Y_MSB)
    z = read_raw_data(Z_MSB)
    
    heading = compute_heading(x, y)
    time.sleep(0.01)
    return heading
