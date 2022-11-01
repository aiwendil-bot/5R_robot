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
    calibrated_architecture = calibration.calibration(rob, nominal_architecture)
    # calibrated_architecture = [-22.49629368, 0.09502097, 22.40372736, 0.05769971, 17.99095831, 18.11306611, 17.889752,
    #                          17.61103216]
    plt.savefig("out/calibration/calibration.png")
    plt.close()
    print("assessing calibration : trying to go to point (5,-20)")
    rob2 = robot.FiveBars(nominal_architecture, mode=0, seed=4, man_var=0.2, mes_var=0.02)
    calibration.assessing_calibration(rob2)
    plt.close()
    input('Calibration finished, press <ENTER> to start path following...')
    # ---------------PATH-FOLLOWING-------------------------------------------------------------

    print("position de départ : (5,-20)")
    print("solutions calculées par IBEX (en utilisant l'architecture calibrée :")
    print(pathfollowing.ibex_solutions)
    print("moyenne de chaque commande : ")
    print(pathfollowing.solutions_initial_point)
    print("calcul de la discrétisation du cercle de centre (0,-20) et de rayon 5 en 100 points")

    architectures = [nominal_architecture, calibrated_architecture]
    colors = ["red", "green"]
    print("pour chaque trajectoire, red = nominal, green = calibrated")
    pathfollowing.main_pathfollowing(architectures, pathfollowing.solutions_initial_point, colors,
                                     pathfollowing.compute_circle_discretisation([0, -20], 5, 100))
    plt.close()
    input("path following finished, press <Enter> to start path planning...")
    pathplanning.main_pathplanning("ibex files/cov files/default_pavage.cov", architectures, colors)
    plt.close()
    if input("path planning precise paving ? y/n") == "y":
        pathplanning.main_pathplanning("ibex files/cov files/precise_pavage.cov", architectures, colors)


main()
