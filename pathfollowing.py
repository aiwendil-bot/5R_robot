import robot
import numpy as np
import scipy
import matplotlib.pyplot as plt

# position de départ : (5,-20)
# solutions calculées par IBEX (en utilisant l'architecture calibrée)

ibex_solutions = [[[-0.9504112839896566, -0.9504112839896549], [3.277549500061275, 3.277549500061278]],
                  [[-0.9504112839896566, -0.9504112839896549], [4.717886729423507, 4.717886729423509]],
                  [[-0.3118221968206622, -0.3118221968206602], [3.277549500061275, 3.277549500061278]],
                  [[-0.3118221968206622, -0.3118221968206602], [4.717886729423507, 4.717886729423509]]]

# on prend la moyenne pour chaque commande :

solutions_initial_point = [[(command[0][0] + command[0][1]) / 2, (command[1][0] + command[1][1]) / 2] for command in
                           ibex_solutions]


def solve(a, x, q0):
    def fun(q):
        return [-a[6] ** 2 + (-a[0] + x[0] - a[4] * np.cos(q[0])) ** 2 + (x[1] - a[4] * np.sin(q[0]) - a[1]) ** 2,
                -a[7] ** 2 + (-a[2] + x[0] - a[5] * np.cos(q[1])) ** 2 + (x[1] - a[5] * np.sin(q[1]) - a[3]) ** 2]

    return scipy.optimize.root(fun, q0).x


def compute_circle_discretisation(center, radius, nb_points):
    discretisation = np.linspace(0, 2 * np.pi, nb_points)
    return [[center[0] + radius * np.cos(t), center[1] + radius * np.sin(t)] for t in discretisation]


def path_following(rob, architecture, sol_initiale, pos, color):
    commands = [sol_initiale]

    for i in range(1, len(pos)):
        q = solve(architecture, pos[i], commands[-1])

        commands.append(q)

    for c in commands:
        rob.actuate([np.degrees(c[0]), np.degrees(c[1])])
        rob.pen_down(color=color)

    rob.pen_up()


def main_pathfollowing(architectures, sol_initiales, colors, positions):
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
