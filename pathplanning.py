import robot
import matplotlib.pyplot as plt
import paving
import numpy as np
from scipy.sparse.csgraph import dijkstra
import scipy
from pathfollowing import path_following

# coordonnées des cercles définissant les obstacles

circles = [(-12, -6, 1.5), (-12, 0, 1.1), (-12, 6, 1.8), (-6, -18, 2.8), (-6, -12, 2.9), (-6, -6, 0.6), (-6, 0, 1.6),
           (-6, 6, 0.4), (-6, 12, 1.9),
           (-6, 18, 0.4), (0, -24, 2.4), (0, -12, 1.9), (0, -6, 0.5), (0, 0, 1.8), (0, 6, 2.9), (0, 12, 1.7),
           (0, 24, 0.5),
           (6, -18, 1.9),
           (6, -12, 2.8), (6, -6, 0.4), (6, 0, 1.9), (6, 6, 0.3), (6, 12, 2.3), (6, 18, 1.2), (12, -6, 2.8),
           (12, 0, 1.3),
           (12, 6, 2.1)]


# générateur des contraintes pour les modèles / plotting circles

def gen_contraintes():
    l = 0
    for (i, j, k) in circles:
        print(f"(x1 - {i} )^2 + (x2 - {j})^2 > {k}^2 ;")
        print(f"c{l} = pyplot.Circle(({i},{j}),{k},color='red',fill=True)")
        print(f"self.ax.add_artist(c{l})")
        l += 1


# nc,l = connected_components(m, directed=False)

# print(nc)
# print(l)

def get_path(predecessors, j):
    path = [j]
    k = j
    while predecessors[k] != -9999:
        path.append(predecessors[k])
        k = predecessors[k]
    return path[::-1]


def solve(a, x, q0):
    def fun(q):
        return [-a[6] ** 2 + (-a[0] + x[0] - a[4] * np.cos(q[0])) ** 2 + (x[1] - a[4] * np.sin(q[0]) - a[1]) ** 2,
                -a[7] ** 2 + (-a[2] + x[0] - a[5] * np.cos(q[1])) ** 2 + (x[1] - a[5] * np.sin(q[1]) - a[3]) ** 2]

    return scipy.optimize.root(fun, q0).x


def milieu(box):
    return [(box.vec[0] + box.vec[1]) / 2, (box.vec[2] + box.vec[3]) / 2, (box.vec[4] + box.vec[5]) / 2,
            (box.vec[6] + box.vec[7]) / 2]


def check_path(paving, path, q0, a):
    commands = [q0]

    [x1, y1, x2, y2, L1, l1, L2, l2] = a
    [q1, q2] = q0
    # elbow1 pose
    e1x, e1y = x1 + L1 * np.cos(q1), y1 + L1 * np.sin(q1)
    # elbow2 pose
    e2x, e2y = x2 + L2 * np.cos(q2), y2 + L2 * np.sin(q2)
    # effector poses
    pose = robot.circles_intersections(e1x, e1y, l1, e2x, e2y, l2)[0]

    x = milieu(paving.boxes[path[0]])[0]
    y = milieu(paving.boxes[path[0]])[1]

    if np.sqrt((pose[0] - x) ** 2 + (pose[1] - y) ** 2) > 0.5:
        print("mauvais départ")
        return False

    for b in path:
        for (x, y, r) in circles:
            if (milieu(paving.boxes[b])[0] - x) ** 2 + (milieu(paving.boxes[b])[1] - y) ** 2 <= r ** 2:
                print("path impossible : obstacle")
                return False
        q = solve(a, milieu(paving.boxes[b]), commands[-1])
        commands.append(q)
        x1 = milieu(paving.boxes[b])[0]
        x2 = milieu(paving.boxes[b])[1]
        [a1, a2, a3, a4, l1, l2, l3, l4] = a
        if (abs((2 * x1 - 2 * a1 - 2 * l1 * np.cos(q[0])) * (2 * (x2 - l2 * np.sin(q[1]) - a4)) - (
                2 * (x1 - a3 - l2 * np.cos(q[1])) * 2 * (x2 - l1 * np.sin(q[0]) - a2))) <= 0.1) or \
                (abs((2 * (-a1 * l1 * np.sin(q[0]) + l1 * a2 * np.cos(q[0]) + l1 * x1 * np.sin(q[0]) - l1 * x2 * np.cos(
                    q[0]))) * (2 * (
                        -a3 * l2 * np.sin(q[1]) + l2 * a4 * np.cos(q[1]) + l2 * x1 * np.sin(q[1]) - l2 * x2 * np.cos(
                    q[1])))) <= 0.1):
            print("path impossible : singularité")
            return False
    return True


def main_pathplanning(cov_file, architectures, colors):
    p = paving.Paving()
    print("chargement du paving...")
    p.from_covfile(cov_file)

    fig1, ax1 = plt.subplots()
    ax1.axis(p.hull([1, 2]))
    p.draw2D(ax1, 1, 2)
    if cov_file == "ibex files/cov files/precise_pavage.cov":
        plt.savefig("out/pathplanning/precise_paving.png")
    else:
        plt.savefig("out/pathplanning/rough_paving.png")

    plt.close()

    fig1, ax1 = plt.subplots()
    ax1.axis(p.hull([3, 4]))
    p.draw2D(ax1, 3, 4)
    if cov_file == "ibex files/cov files/precise_pavage.cov":
        plt.savefig("out/pathplanning/precise_paving_commands.png")
    else:
        plt.savefig("out/pathplanning/rough_paving_commands.png")

    plt.close()

    boxes_depart = []
    boxes_arrivee = []

    print("boxes de départ possibles :")
    for b in p.boxes_intersecting([0, -15, -np.pi, 0], [0, -15, np.pi, 2 * np.pi]):
        boxes_depart.append(b)
        print(p.boxes[b])

    print("boxes d'arrivée possibles :")
    for b in p.boxes_intersecting([0, 15, -np.pi, 0], [0, 15, np.pi, 2 * np.pi]):
        boxes_arrivee.append(b)
        print(p.boxes[b])

    adj = p.adjacency_matrix()

    possibles_paths = []
    # filtrage des paths possibles (le centre des boites ne doit pas se trouver à l'intérieur d'un obstacle
    print("calcul des plus courts chemins et vérification de leurs faisabilité...")
    for b1 in boxes_depart:
        dist_matrix, predecessors = dijkstra(csgraph=adj, directed=False, indices=b1, return_predecessors=True)
        for b2 in boxes_arrivee:
            candid = get_path(predecessors, b2)
            flag = True
            for architecture in architectures:
                if not check_path(p, candid, milieu(p.boxes[candid[0]])[2:], architecture):
                    flag = False
            if flag:
                possibles_paths.append(candid)

    print(p.boxes[possibles_paths[-1][0]])
    print(milieu(p.boxes[possibles_paths[-1][0]]))
    print("nombre de paths possibles : ", len(possibles_paths))
    # possibles_paths = [possibles_paths[-1]]

    if cov_file == "ibex files/cov files/precise_pavage.cov":
        possibles_paths = possibles_paths[0:1]
    for indice_path in range(len(possibles_paths)):
        print(f"using possible path n°{indice_path + 1}...")
        rob = robot.FiveBars(architectures[0], mode=0, seed=4, man_var=0.2, mes_var=0.02)
        fig1, ax1 = rob._fig, rob.ax
        ax1.axis(p.hull([1, 2]))
        sp0 = p.subpaving(possibles_paths[indice_path])
        sp0.draw2D(ax1, 1, 2, ec=None, fc='yellow')
        for (x, y, r) in circles:
            rob.ax.add_artist(plt.Circle((x, y), r, color='red', fill=True))
        traj = []
        for index in possibles_paths[indice_path]:
            traj.append(milieu(p.boxes[index]))
            rob.ax.plot(milieu(p.boxes[index])[0], milieu(p.boxes[index])[1], color='black', marker='*', markersize=0.5)
        for k in range(len(architectures)):
            name = "nominal" if k == 0 else "calibrated"
            print(f"using {name} architecture...")
            print(milieu(p.boxes[possibles_paths[indice_path][0]])[2:])
            path_following(rob, architectures[k], milieu(p.boxes[possibles_paths[indice_path][0]])[2:], traj, colors[k])
        if cov_file == "ibex files/cov files/precise_pavage.cov":
            plt.savefig(f"out/pathplanning/precise/path_{indice_path + 1}.png")
        else:
            plt.savefig(f"out/pathplanning/default/path_{indice_path + 1}.png")
