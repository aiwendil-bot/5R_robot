import robot
import matplotlib.pyplot as plt
import paving
import numpy as np
from scipy.sparse.csgraph import dijkstra
import scipy
from pathfollowing import path_following

# coordonnées des cercles définissant les obstacles

circles = [(-12, -6, 1.5), (-12, 0, 1.1), (-12, 6, 1.8), (-6, -18, 2.8), (-6, -12, 2.9), (-6, -6, 0.6), (-6, 0, 1.6),
           (-6, 6, 0.4), (-6, 12, 1.9), (-6, 18, 0.4), (0, -24, 2.4), (0, -12, 1.9), (0, -6, 0.5), (0, 0, 1.8),
           (0, 6, 2.9), (0, 12, 1.7), (0, 24, 0.5), (6, -18, 1.9), (6, -12, 2.8), (6, -6, 0.4), (6, 0, 1.9),
           (6, 6, 0.3), (6, 12, 2.3), (6, 18, 1.2), (12, -6, 2.8), (12, 0, 1.3), (12, 6, 2.1)]


# générateur des contraintes pour les modèles / plotting circles
def gen_contraintes():
    l = 0
    for (i, j, k) in circles:
        print(f"(x1 - {i} )^2 + (x2 - {j})^2 > {k}^2 ;")
        print(f"c{l} = pyplot.Circle(({i},{j}),{k},color='red',fill=True)")
        print(f"self.ax.add_artist(c{l})")
        l += 1


# récupère le chemin depuis la liste des prédecesseurs
def get_path(predecessors, j):
    path = [j]
    k = j
    while predecessors[k] != -9999:
        path.append(predecessors[k])
        k = predecessors[k]
    return path[::-1]


# solve permet d'isoler l'appel à optimize.root
# paramètres : l'architecture, la position à atteindre et la commande d'où commencer la résolution
def solve(a, x, q0):
    def fun(q):
        return [-a[6] ** 2 + (-a[0] + x[0] - a[4] * np.cos(q[0])) ** 2 + (x[1] - a[4] * np.sin(q[0]) - a[1]) ** 2,
                -a[7] ** 2 + (-a[2] + x[0] - a[5] * np.cos(q[1])) ** 2 + (x[1] - a[5] * np.sin(q[1]) - a[3]) ** 2]

    return scipy.optimize.root(fun, q0).x


# calcul le centre (moyenne des coordonnées et des commandes) d'une boîte
def milieu(box):
    return [(box.vec[0] + box.vec[1]) / 2, (box.vec[2] + box.vec[3]) / 2, (box.vec[4] + box.vec[5]) / 2,
            (box.vec[6] + box.vec[7]) / 2]


# vérifie plusieurs critères pour s'assurer qu'un chemin est valide pour un mode du robot
# les calculs sont faits avec l'architecture calibrée

def check_path(paving, path, a, mode):
    [x1, y1, x2, y2, L1, l1, L2, l2] = a
    poses = []

    # pour chaque boite, on vérifie
    #   -> si les commandes de celle ci ne conduisent pas à une singularité
    #   -> si est bien atteignable pour le mode (en paramètre) en utilisant les commandes de la boite
    #   -> si le centre de la boite n'est pas dans un obstacle
    #   -> si les commandes calculées de proche en proche (via root, comme dans le path following) ne conduisent
    #      pas à une singularité
    for i in range(len(path)):
        [q1, q2] = milieu(paving.boxes[path[i]])[2:]
        # elbow1 pose
        e1x, e1y = x1 + L1 * np.cos(q1), y1 + L1 * np.sin(q1)
        # elbow2 pose
        e2x, e2y = x2 + L2 * np.cos(q2), y2 + L2 * np.sin(q2)
        # effector poses
        poses.append(robot.circles_intersections(e1x, e1y, l1, e2x, e2y, l2))
        if len(robot.circles_intersections(e1x, e1y, l1, e2x, e2y, l2)) != 2:
            print("path impossible : singularité")
            return False

        if scipy.spatial.distance.euclidean(poses[i][mode],
                                            milieu(paving.boxes[path[i]])[
                                            :2]) > 0.3 or scipy.spatial.distance.euclidean(
            poses[i][mode], milieu(paving.boxes[path[i]])[:2]) > 0.5:
            print("path impossible pour le mode ", mode)
            return False

        for (x, y, r) in circles:
            if (milieu(paving.boxes[path[i]])[0] - x) ** 2 + (milieu(paving.boxes[path[i]])[1] - y) ** 2 <= r ** 2:
                print("path impossible : obstacle")
                return False

    commands = [milieu(paving.boxes[path[0]])[2:]]
    for i in range(1, len(path)):
        [q1, q2] = solve(a, milieu(paving.boxes[path[i]])[:2], commands[-1])
        commands.append([q1, q2])
        e1x, e1y = x1 + L1 * np.cos(q1), y1 + L1 * np.sin(q1)
        # elbow2 pose
        e2x, e2y = x2 + L2 * np.cos(q2), y2 + L2 * np.sin(q2)
        if len(robot.circles_intersections(e1x, e1y, l1, e2x, e2y, l2)) != 2:
            print("path impossible : singularité")
            return False

    return True


def main_pathplanning(cov_file, architectures):
    # en rouge, on affiche le chemin calculé avec l'architecture nominale, en vert avec l'architecture calibrée
    colors = ["red", "green"]
    p = paving.Paving()
    print("chargement du paving...")
    p.from_covfile(cov_file)

    # on sauvegarde les pavings (coordonnées et commandes)

    fig1, ax1 = plt.subplots()
    ax1.axis(p.hull([1, 2]))
    p.draw2D(ax1, 1, 2)

    plt.savefig("out/pathplanning/paving.png")

    plt.close()

    fig1, ax1 = plt.subplots()
    ax1.axis(p.hull([3, 4]))
    p.draw2D(ax1, 3, 4)
    plt.savefig("out/pathplanning/paving_commands.png")

    plt.close()

    boxes_depart = []
    boxes_arrivee = []

    # on utilise boxes intersecting pour déterminer quelles boites contiennent les points de départ et d'arrivée
    print("boxes de départ possibles :")
    for b in p.boxes_intersecting([0, -15, -np.pi, 0], [0, -15, np.pi, 2 * np.pi]):
        boxes_depart.append(b)
        print(p.boxes[b])

    print("boxes d'arrivée possibles :")
    for b in p.boxes_intersecting([0, 15, -np.pi, 0], [0, 15, np.pi, 2 * np.pi]):
        boxes_arrivee.append(b)
        print(p.boxes[b])

    # matrice d'adjacence pour calculer les plus courts chemins
    adj = p.adjacency_matrix()

    possibles_paths = [[], []]
    # filtrage des paths possibles (le centre des boites ne doit pas se trouver à l'intérieur d'un obstacle
    # on considère les chemins entre chaque paire (boite départ, boite arrivée)
    # on sépare les chemins selon le mode pour lequel ils sont valables
    print("calcul des plus courts chemins et vérification de leurs faisabilité...")
    for b1 in boxes_depart:
        dist_matrix, predecessors = dijkstra(csgraph=adj, directed=False, indices=b1, return_predecessors=True)
        for b2 in boxes_arrivee:
            candid = get_path(predecessors, b2)
            for mode in range(2):
                if check_path(p, candid, architectures[1], mode):
                    possibles_paths[mode].append(candid)

    print("nombre de paths possibles : ", len(possibles_paths[0]) + len(possibles_paths[1]))

    for mode in range(2):
        print(f"using mode {mode}...")
        for indice_path in range(len(possibles_paths[mode])):
            print(f"using possible path n°{indice_path + 1}...")
            # on recrée un robot à chaque chemin
            rob = robot.FiveBars(architectures[0], mode=mode, seed=4, man_var=0.2, mes_var=0.02)
            fig1, ax1 = rob._fig, rob.ax
            ax1.axis(p.hull([1, 2]))
            # on affiche les boxes du chemin en jaune
            sp0 = p.subpaving(possibles_paths[mode][indice_path])
            sp0.draw2D(ax1, 1, 2, ec=None, fc='yellow')
            # on affiche les obstacles en bleu
            for (x, y, r) in circles:
                rob.ax.add_artist(plt.Circle((x, y), r, color='blue', fill=True))
            traj = []
            # on affiche les centres des boites en noir (ce sont les points à atteindre pour le robot)
            for index in possibles_paths[mode][indice_path]:
                traj.append(milieu(p.boxes[index])[:2])
                rob.ax.plot(milieu(p.boxes[index])[0], milieu(p.boxes[index])[1], color='black', marker='*',
                            markersize=0.5)
            # on fait le chemin pour chaque architecture (on réutilise la fonction path_following)
            for k in range(len(architectures)):
                name = "nominal" if k == 0 else "calibrated"
                print(f"using {name} architecture...")
                path_following(rob, architectures[k], milieu(p.boxes[possibles_paths[mode][indice_path][0]])[2:], traj,
                               colors[k])
            plt.savefig(f"out/pathplanning/path_mode_{mode}_{indice_path + 1}.png")
