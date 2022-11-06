import numpy
import matplotlib.pyplot as plt

import robot


# returns 5R kinematic functions
def f_5R(architecture,pose,command):
    [a1,a2,a3,a4,a5,a6,a7,a8] = architecture
    [x1,x2] = pose
    [q1,q2] = numpy.radians(command)
    f1 = -a6**2 + (-a1 + x1 - a5*numpy.cos(q1))**2 + (x2 - a5*numpy.sin(q1) - a2)**2
    f2 = -a8**2 + (-a3 + x1 - a7*numpy.cos(q2))**2 + (x2 - a7*numpy.sin(q2) - a4)**2
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
    commands1 = [[q1,q2] for q1 in range(0,91,10) for q2 in range(135,181,10)]
    measures1 = make_measurements(rob,commands1,col='red')
    calibrated_architecture1 = calibrate(f_5R,nominal_architecture,measures1)
    commands2 = [[q1,q2] for q1 in range(0,46,10) for q2 in range(90,181,10)]
    measures2 = make_measurements(rob,commands2,col='red')
    calibrated_architecture2 = calibrate(f_5R,calibrated_architecture1,measures2)

    print("calibrated architecture computed :", calibrated_architecture2)
    return calibrated_architecture2

def assessing_calibration():

    #ibex solutions to get to point (5,-20)
    ibex_solutions = [[[-0.9706922251069474, -0.9706922251069458], [3.278947809944586, 3.278947809944588]],
                  [[-0.9706922251069474, -0.9706922251069458],[4.716488419540196, 4.716488419540199]],
                  [[-0.291541255703371, -0.2915412557033691], [3.278947809944586, 3.278947809944588]],
                  [[-0.291541255703371, -0.2915412557033691], [4.716488419540196, 4.716488419540199]]]

# on prend la moyenne pour chaque commande :

    solutions = [[(command[0][0] + command[0][1]) / 2, (command[1][0] + command[1][1]) / 2] for command in
                           ibex_solutions]

    #ibex solutions to get to point (5,20)

    ibex_solutions2 = [[[0.2777791892666854, 0.2777791892666872], [1.565918675729136, 1.565918675729138]],
    [[0.2777791892666854, 0.2777791892666872], [3.010730358302635, 3.010730358302638]],
    [[0.9754140743428796, 0.9754140743428813], [1.565918675729136, 1.565918675729138]],
    [[0.9754140743428796, 0.9754140743428813], [3.010730358302635, 3.010730358302638]]]

    solutions2 = [[(command[0][0] + command[0][1]) / 2, (command[1][0] + command[1][1]) / 2] for command in
                 ibex_solutions2]

    initial_solutions = [solutions,solutions2]
    for i in range(len(initial_solutions)):
        #pour atteindre le point (5,20), le robot doit être en mode 1
        rob = robot.FiveBars([-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8, 17.8], mode=i, seed=4, man_var=0.2, mes_var=0.02)
        print("mode du robot : ", i)
        rob.ax.plot([5],[-20],color='black',marker='*')
        rob.ax.plot([5],[20],color='black',marker='*')
        colors = ['blue','green','yellow', 'red']
        for s in range(len(solutions)):
            rob.pen_up()
            rob.go_home()
            rob.pen_down(color=colors[s])
            rob.actuate([numpy.degrees(initial_solutions[i][s][0]), numpy.degrees(initial_solutions[i][s][1])])
            print("point atteint : ",rob.measure_pose())
            print(numpy.radians(rob.measure_command()[0]),numpy.radians(rob.measure_command()[1]))
        plt.savefig(f"out/calibration/assessed_calibration{i+1}.png")
        plt.close()