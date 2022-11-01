import numpy
import matplotlib.pyplot as plt

# returns 5R kinematic functions
def f_5R(architecture,pose,command):
    [a1,a2,a3,a4,a5,a6,a7,a8] = architecture
    [x1,x2] = pose
    [q1,q2] = numpy.radians(command)
    f1 = -a7**2 + (-a1 + x1 - a5*numpy.cos(q1))**2 + (x2 - a5*numpy.sin(q1) - a2)**2
    f2 = -a8**2 + (-a3 + x1 - a6*numpy.cos(q2))**2 + (x2 - a6*numpy.sin(q2) - a4)**2
    return [f1,f2]

#------------------------------------------------
# Actuation of the robot in order to generate measures for calibration
def make_measurements(robot,commands,col='black',mar='*'):
    robot.actuate(commands[0])
    if col!='black':
        robot.pen_down(col)
    measures=[]
    print('   Taking measures ...')
    for q in commands:
        robot.actuate(q)
        x = robot.measure_pose()
        robot.ax.plot([x[0]],[x[1]],color=col,marker=mar)
        measures.append((x,q))
    if col!='black':
        robot.pen_up()
    robot.go_home()
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

def calibration(rob, nominal_architecture):


    rob.actuate([0,90])
    commands = [[q1,q2] for q1 in range(0,91,10) for q2 in range(135,181,10)] + [[q1,q2] for q1 in range(0,46,10) for q2 in range(90,181,10)]
    measures = make_measurements(rob,commands,col='red')
    calibrated_architecture = calibrate(f_5R,nominal_architecture,measures)
    print("calibrated architecture computed :", calibrated_architecture)
    return calibrated_architecture

def assessing_calibration(rob):

    #ibex solutions to get to point (5,-20)
    solutions = [[-0.9504112839896566,3.277549500061275],[-0.9504112839896566,4.717886729423507],
                 [-0.3118221968206622, 3.277549500061275],[-0.3118221968206622,4.717886729423507]]
    rob.ax.plot([5],[-20],color='black',marker='*')
    colors = ['blue','green','yellow', 'red']
    for s in range(len(solutions)):
        rob.pen_up()
        rob.go_home()
        rob.pen_down(color=colors[s])
        rob.actuate([numpy.degrees(solutions[s][0]), numpy.degrees(solutions[s][1])])
        print("point atteint : ",rob.measure_pose())
        rob.refresh()
    plt.savefig("out/calibration/assessed_calibration.png")


