import robot
import numpy as np
import scipy
import matplotlib.pyplot as plt

# position de départ : (5,-20)
# solutions calculées par IBEX (en utilisant l'architecture calibrée)
# modèle : assessing_calib.mbx (cf calibration)

ibex_solutions = [[[-0.9706922251069474, -0.9706922251069458], [3.278947809944586, 3.278947809944588]],
                  [[-0.9706922251069474, -0.9706922251069458],[4.716488419540196, 4.716488419540199]],
                  [[-0.291541255703371, -0.2915412557033691], [3.278947809944586, 3.278947809944588]],
                  [[-0.291541255703371, -0.2915412557033691], [4.716488419540196, 4.716488419540199]]]



# on prend la moyenne pour chaque commande :

solutions_initial_point = [[(command[0][0] + command[0][1]) / 2, (command[1][0] + command[1][1]) / 2] for command in
                           ibex_solutions]

# solve permet d'isoler l'appel à optimize.root
# paramètres : l'architecture, la position à atteindre et la commande d'où commencer la résolution
def solve(a, x, q0):

    #équations kinetic du 5R, uniquement en fonction de q
    def fun(q):
        return [-a[5] ** 2 + (-a[0] + x[0] - a[4] * np.cos(q[0])) ** 2 + (x[1] - a[4] * np.sin(q[0]) - a[1]) ** 2,
                -a[7] ** 2 + (-a[2] + x[0] - a[6] * np.cos(q[1])) ** 2 + (x[1] - a[6] * np.sin(q[1]) - a[3]) ** 2]

    return scipy.optimize.root(fun, q0).x

#discretise le cercle (center, radius) en un nombre nb_points de points
def compute_circle_discretisation(center, radius, nb_points):
    discretisation = np.linspace(0, 2 * np.pi, nb_points)
    return [[center[0] + radius * np.cos(t), center[1] + radius * np.sin(t)] for t in discretisation]

#fait tracer en 'color' le robot 'rob' les points de la liste 'pos'
#calculés avec l'architecture, partant de la 'sol_initiale'
#fait appel à solve pour calculer les commandes pour chaque point de pos
def path_following(rob, architecture, sol_initiale, pos, color):
    commands = [sol_initiale]

    for i in range(1, len(pos)):
        q = solve(architecture, pos[i], commands[-1])

        commands.append(q)

    for c in commands:
        rob.actuate([np.degrees(c[0]), np.degrees(c[1])])
        rob.pen_down(color=color)

    rob.pen_up()

#effectue le path following des points de 'positions' en les affichant au préalable
#et le fait pour chacune des 'solutions initiales' en différentes couleurs pour les deux architectures
def main_pathfollowing(architectures, sol_initiales, positions):
    colors = ["red", "green"]
    for i in range(len(sol_initiales)):
        print(f"using initial solution n°{i + 1}...")
        rob = robot.FiveBars(architectures[0], mode=0, seed=4, man_var=0.2, mes_var=0.02)
        for point in positions:
            rob.ax.plot(point[0], point[1], color='black', marker='*', markersize=0.2)
        for k in range(len(architectures)):
            name = "nominal" if k == 0 else "calibrated"
            print(f"using {name} architecture...")
            path_following(rob, architectures[k], sol_initiales[i], positions, colors[k])
        plt.savefig(f"out/pathfollowing/solution_{i + 1}.png")
