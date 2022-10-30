#################################################
# TwoBars robot demo
import math
import numpy

# >>> from importlib import reload
# >>> import calibration_demo,robot

#################################################
# Definition of a RR robot with approximate architecture
import robot



## Actuating and measuring
# r.actuate([0,90])
# r.measure_pose()
# r.measure_pose()
# r.measure_command()
# r.measure_command()

# ## Tracing pose
# r.go_home()
# r.pen_down()
# r.actuate([0,90])
# r.actuate([90,360])
# r.pen_up()

# Invalid commands
# r.actuate([180,180])
# r.actuate([120,180])
# r.actuate([180,180])

# Approximate robots


#################################################
# CALIBRATION

#------------------------------------------------
# RR kinematic functions
def f_5R(architecture,pose,command):
    [a1,a2,a3,a4,a5,a6,a7,a8] = architecture
    [x1,x2] = pose
    [q1,q2] = numpy.radians(command)
    f1 = -a7**2 + (-a1 + x1 - a5*numpy.cos(q1))**2 + (x2 - a5*numpy.sin(q1) - a2)**2
    f2 = -a8**2 + (-a3 + x1 - a6*numpy.cos(q2))**2 + (x2 - a6*numpy.sin(q2) - a4)**2
    return [f1,f2]

#------------------------------------------------
# Actuation of the robot in order to generate measures for calibration
def make_measurements(r2,commands,col='black',mar='*'):
    r2.actuate(commands[0])
    if col!='black':
        r2.pen_down(col)
    measures=[]
    print('   Taking measures ...')
    for q in commands:
        r2.actuate(q)
        x = r2.measure_pose()
        r2.ax.plot([x[0]],[x[1]],color=col,marker=mar)
        measures.append((x,q))
    if col!='black':
        r2.pen_up()
    r2.go_home()
    return measures

#------------------------------------------------
# calibration from measurements
from scipy.optimize import least_squares
def calibrate(kinematic_functions,nominal_architecture,measures):
    # error function ----
    def errors(a):
        err=[]
        for (x,q) in measures:
            for fi in kinematic_functions(a,x,q):
                err.append(fi)
        return err
    # -------------------
    print('   Calibration processing ...')
    sol = least_squares(errors,nominal_architecture)
    print('   status : ',sol.message)
    print('   error : ',sol.cost)
    print('   result : ',sol.x)
    return sol.x

#------------------------------------------------
# Calibration : first choice of commands and resulting measurements

def main_calibration():
    nominal_architecture = [-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8, 17.8]
    r = robot.FiveBars(nominal_architecture,mode=0,seed=4,man_var=0.2,mes_var=0.02)
    r.actuate([0,90])
    commands = [[q1,q2] for q1 in range(0,91,10) for q2 in range(135,181,10)] + [[q1,q2] for q1 in range(0,46,10) for q2 in range(90,181,10)]
    measures = make_measurements(r,commands,col='red')
    calibrated_architecture = calibrate(f_5R,nominal_architecture,measures)
    return calibrated_architecture

#------------------------------------------------
#Calibration : second choice of commands and resulting measurements
#commands = [[q1,q2] for q1 in range(0,46,10) for q2 in range(90,181,10)]
#measures = make_measurements(r,commands,col='blue',mar='o')
#calibrated_architecture = calibrate(f_5R,nominal_architecture,measures)
#print(r.get_architecture())