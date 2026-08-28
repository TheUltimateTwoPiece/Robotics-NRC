from movement_functions import *
 
# def collect():
#     straight_gyro(121, 130)
#     arm.run_target(300,0)
#     wait(1000)
#     claw.run(300)
#     wait(1000)
#     arm.run_target(300,50)
#     straight_gyro(-126, 300)
#     wait(1000)

def collect():
    straight_gyro(81, 130)
    arm.run_target(300,0)
    wait(1000)
    claw.run(300)
    wait(1000)
    straight_gyro(-50, 130)
    claw.run(-300)
    wait(1000)
    straight_gyro(60, 130)
    wait(1000)
    claw.run(300)
    wait(1000)
    arm.run_target(300,50)
    straight_gyro(-91, 300)
    wait(1000)
 
def stack_l2():
    arm.run_target(300, 125)
    straight_gyro(80, 200)
    wait(1000)
    claw.run(-300)
    wait(1000)
    straight_gyro(-85, 200)
    arm.run_target(300, 0)
 
def stack_l3():
    arm.run_target(300, 180)
    straight_gyro(113, 200)
    wait(1000)
    claw.run(-300)
    wait(1000)
    straight_gyro(-95, 200)
    arm.run_target(300, 0)
 
def align_zone():
    straight_gyro(30, 100)
    PDtrack_and_junction(100, 30, 1.3, 85)
    straight_gyro(73, 100)
    
 
def stack_block(collect_area, stack_area, layer):
    #assumes that robot is positioned exactly in the middle of the circle
    returnlist = []
    claw.run(-300)
    
    if collect_area == 0:
        returnlist.append(0)
    elif collect_area == 1:
        returnlist.append(-95)
        turn_gyro(95, 250)
    elif collect_area == -1:
        turn_gyro(-95, 250)
        returnlist.append(95)
    


    align_zone()   
    collect()
 
   
    turn_gyro(returnlist[0], 250)
    if stack_area == 0:
        returnlist.append(0)
    elif stack_area == 1:
        turn_gyro(96, 250)
        returnlist.append(-94)
    elif stack_area == -1:
        turn_gyro(-95, 250)
        returnlist.append(95)
 

 
    if layer == 2:
        stack_l2()
    elif layer == 3:
        stack_l3()
 
    turn_gyro(returnlist[1], 250)
 
    