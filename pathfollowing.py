import robot
import numpy as np
import scipy

import calibration

#calibrated_architecture = calibration.main_calibration()
calibrated_architecture =[-22.49629368,0.09502097,22.40372736,0.05769971,17.99095831,18.11306611,17.889752,17.61103216]
nominal_architecture = [-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8, 17.8]
r = robot.FiveBars(nominal_architecture,mode=0,seed=4,man_var=0.2,mes_var=0.02)

#position de départ : (5,-20)

solution_nom = [[-0.9293947977378646,3.265547488610673], [-0.9293947977378646, 4.721570472915458],
                [-0.328197775093003, 3.265547488610673], [-0.328197775093003, 4.721570472915456]]

solutions_calib = [[-0.9504112839896566, 3.277549500061275] , [-0.9504112839896566, 4.717886729423507],
                   [-0.3118221968206622,  3.277549500061275], [-0.3118221968206622 , 4.717886729423507]]

#r.actuate([np.degrees(solution4[0]),np.degrees(solution4[1])])

discretisation = np.linspace(0, 2*np.pi, 100)
radius = 5

positions = [[0 + radius*np.cos(t),-20 + radius * np.sin(t)] for t in discretisation]

def solve(a,x,q0):
    def fun(q):
        return [-a[6]**2 + (-a[0] + x[0] - a[4]*np.cos(q[0]))**2 + (x[1] - a[4]*np.sin(q[0]) - a[1])**2,
                -a[7]**2 + (-a[2] + x[0] - a[5]*np.cos(q[1]))**2 + (x[1] - a[5]*np.sin(q[1]) - a[3])**2]

    return scipy.optimize.root(fun,q0).x


def path_following(robot,solnom,solcalib,pos):
    commands_nom = [solnom]
    commands_calib = [solcalib]

    for i in range(1,len(pos)):
        commands_calib.append(solve(calibrated_architecture,pos[i],commands_calib[-1]))
        commands_nom.append(solve(nominal_architecture,pos[i],commands_nom[-1]))

    for c in commands_calib:
        robot.actuate([np.degrees(c[0]),np.degrees(c[1])])
        robot.pen_down(color='green')
        
    robot.pen_up()


    for c in commands_nom:
        robot.actuate([np.degrees(c[0]),np.degrees(c[1])])
        robot.pen_down(color='blue')


#path_following(r,solution_nom[0],solutions_calib[0],positions)