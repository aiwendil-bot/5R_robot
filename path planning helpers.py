import robot
from matplotlib import pyplot

import paving
import numpy as np
from scipy.sparse.csgraph import dijkstra
import scipy
from pathfollowing import path_following

data = [(-12, -6, 1.5), (-12, 0, 1.1), (-12, 6, 1.8), (-6, -18, 2.8), (-6, -12, 2.9), (-6, -6, 0.6), (-6, 0, 1.6),
        (-6, 6, 0.4), (-6, 12, 1.9),
        (-6, 18, 0.4), (0, -24, 2.4), (0, -12, 1.9), (0, -6, 0.5), (0, 0, 1.8), (0, 6, 2.9), (0, 12, 1.7), (0, 24, 0.5),
        (6, -18, 1.9),
        (6, -12, 2.8), (6, -6, 0.4), (6, 0, 1.9), (6, 6, 0.3), (6, 12, 2.3), (6, 18, 1.2), (12, -6, 2.8), (12, 0, 1.3),
        (12, 6, 2.1)]

l = 3
for (i, j, k) in data:
    # print(f"(x1 - {i} )^2 + (x2 - {j})^2 > {k}^2 ;")
    # print(f"c{l} = pyplot.Circle(({i},{j}),{k},color='red',fill=True)")
    # print(f"self.ax.add_artist(c{l})")
    l += 1

nominal_architecture = [-22.5, 0, 22.5, 0, 17.8, 17.8, 17.8, 17.8]
rob = robot.FiveBars(nominal_architecture, mode=0, seed=4, man_var=0.2, mes_var=0.02)
calibrated_architecture = [-22.49629368, 0.09502097, 22.40372736, 0.05769971, 17.99095831, 18.11306611, 17.889752,
                           17.61103216]

p = paving.Paving()
p.from_covfile("5R_calib2.cov")

# fig1.show()
# pyplot.savefig("test2.png")

from scipy.sparse.csgraph import connected_components

# nc,l = connected_components(m, directed=False)

# print(nc)
# print(l)
# print(p.boxes)

boxes_depart = []
boxes_arrivee = []

for b in p.boxes_intersecting([0, -15, -np.pi, 0], [0, -15, np.pi, 2 * np.pi]):
    boxes_depart.append(b)

for b in p.boxes_intersecting([0, 15, -np.pi, 0], [0, 15, np.pi, 2 * np.pi]):
    boxes_arrivee.append(b)

adj = p.adjacency_matrix()


def get_path(Pr, j):
    path = [j]
    k = j
    while Pr[k] != -9999:
        path.append(Pr[k])
        k = Pr[k]
    return path[::-1]


def solve(a, x, q0):
    def fun(q):
        return [-a[6] ** 2 + (-a[0] + x[0] - a[4] * np.cos(q[0])) ** 2 + (x[1] - a[4] * np.sin(q[0]) - a[1]) ** 2,
                -a[7] ** 2 + (-a[2] + x[0] - a[5] * np.cos(q[1])) ** 2 + (x[1] - a[5] * np.sin(q[1]) - a[3]) ** 2]

    return scipy.optimize.root(fun, q0).x


possibles_paths = []
# dist_matrix, predecessors = dijkstra(csgraph=adj, directed=False, indices=boxes_depart, return_predecessors=True)

print(boxes_depart)
print(boxes_arrivee)


def milieu(box):
    return [(box.vec[0] + box.vec[1]) / 2, (box.vec[2] + box.vec[3]) / 2]


def check_path(path):
    for b in path:
        for (x, y, r) in data:
            if (milieu(p.boxes[b])[0] - x) ** 2 + (milieu(p.boxes[b])[1] - y) ** 2 <= r ** 2:
                return False
    return True


for b1 in boxes_depart:
    dist_matrix, predecessors = dijkstra(csgraph=adj, directed=False, indices=b1, return_predecessors=True)
    for b2 in boxes_arrivee:
        candid = get_path(predecessors, b2)
        if check_path(candid):
            possibles_paths.append(candid)

print(len(possibles_paths))

# calculés avec ibex, 1 et 2 donnent des pathings faux
solutioncalib = [[-1.303631677958944, 3.031248955019107], [-1.303631677958944, 4.435455286422952],
                 [0.1216379798564773, 3.031248955019107], [0.1216379798564773, 4.435455286422952]]

commands_calib = solutioncalib[2]

solutionnom = [[-1.296106468894168, 3.02149139179076], [-1.296106468894168, 4.437699122483959],
               [0.1201012617990314, 3.02149139179076], [0.1201012617990314, 4.437699122483959]]


for indice_path in range(len(possibles_paths)):
    for s in range(len(solutioncalib)):
        rob = robot.FiveBars(nominal_architecture, mode=0, seed=4, man_var=0.2, mes_var=0.02)
        fig1, ax1 = rob._fig, rob.ax
        #p.draw2D(ax1, 1, 2)
        ax1.axis(p.hull([1, 2]))
        sp0 = p.subpaving(possibles_paths[indice_path])
        sp0.draw2D(ax1, 1, 2, ec=None, fc='yellow')
        #pyplot.savefig(f"pathprecis{k}.png")
        traj = []
        for index in possibles_paths[indice_path]:
            traj.append(milieu(p.boxes[index]))
            print(milieu(p.boxes[index]))

        path_following(rob,solutionnom[s],solutioncalib[s],traj)
        pyplot.savefig(f"path_{indice_path}_initialsol_{s}.png")
#
