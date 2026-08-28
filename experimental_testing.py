from block_functions import *
from movement_functions import *

claw.run(-300)

turn_gyro(180)
PDtrack_and_junction(100, 30, 1, 75)
straight_gyro(73, 100)
straight_gyro(500, 200)

turn_gyro(-90, 250)
align_zone()   
collect()
turn_gyro(180)
align_zone()
stack_l2()