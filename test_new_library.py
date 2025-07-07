import new_library as lp #LittlePro

#test start / stop motors:
#start motors os speed +-600 for 1.2 sec with force-stop:
'''
lp.motor_start(600,600)
lp.time.sleep(1.2)
lp.motor_stop(1)


lp.motor_start(-600,-600)
lp.time.sleep(1.2)
lp.motor_stop(1)
'''

#test motor position move:
#start motors on speed +- 300 for 3000 positions
'''
lp.motor_position_move(300, 3000)
lp.time.sleep(1)
lp.motor_position_move(300, 3000)
lp.time.sleep(1)

lp.motor_position_move(-300, 3000)
lp.time.sleep(1)
lp.motor_position_move(-300, 3000)
'''

#test motor position turn:
#turn motors on speed +- 100 for 3000 positions
'''
lp.motor_position_turn(100, 3000)
lp.motor_position_turn(-100, 3000)
'''

#test motor degrees turn:
#turn motors on speed +- 100 for 90 | 45 degrees 
'''
lp.motor_degrees_turn(100, 90)
lp.motor_degrees_turn(-100, 90)

lp.motor_degrees_turn(100, 45)
lp.motor_degrees_turn(-100, 45)
'''
