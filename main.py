import numpy as np

import calibration
import pathplanning
import robot
import matplotlib.pyplot as plt
import pathfollowing


def main():
    # ---------------CALIBRATION-----------------------------------------------------------------

    nominal_architecture = [-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8, 17.8]
    rob = robot.FiveBars(nominal_architecture, mode=0, seed=4, man_var=0.2, mes_var=0.02)

    input('Press <ENTER> to start calibration...')
    #calibrated_architecture = calibration.calibration(rob, nominal_architecture)
    #sauvegarde de l'architecture calibrée pour ne pas refaire la calibration lors des tests
    calibrated_architecture = [-22.49629368, 0.09502097, 22.40372736, 0.05769971, 17.99095831, 18.11306611, 17.889752,
                             17.61103216]
    calibrated_architecture = [-22.4827476,    0.09657084,  22.40045036,   0.0493766,   17.97896946,
    17.88814187,  18.11218982,  17.60076237]
    plt.savefig("out/calibration/calibration.png")
    plt.close()
    print("assessing calibration : trying to go to point (5,-20)")
    #cf modèle "assessing_calib.mbx"

    #calibration.assessing_calibration()
    plt.close()
    input('Calibration finished, press <ENTER> to start path following...')
    # ---------------PATH-FOLLOWING-------------------------------------------------------------

    print("position de départ : (5,-20)")
    print("solutions calculées par IBEX (en utilisant l'architecture calibrée) :")
    print(pathfollowing.ibex_solutions)
    print("moyenne de chaque commande : ")
    print(pathfollowing.solutions_initial_point)
    architectures = [nominal_architecture, calibrated_architecture]
    print("pour chaque trajectoire, red = nominal, green = calibrated")
    #pathfollowing.main_pathfollowing(architectures, pathfollowing.solutions_initial_point,
      #                               pathfollowing.compute_circle_discretisation([0, -20], 5, 100))
    plt.close()
    input("path following finished, press <Enter> to start path planning...")
    #voir les modèles 5R_pavage.mbx et 5R_pavage_singularités.mbx
    pathplanning.main_pathplanning("ibex files/cov files/5R_pavage_singularités.cov", architectures)
    plt.close()


main()
