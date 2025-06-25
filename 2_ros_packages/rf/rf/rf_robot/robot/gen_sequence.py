import numpy as np

# Assumed environment
#            8
#            |
# 1 -------- 2 - 3 ----- 4
# |              |       |
# 5 ------------ 6 ----- 7
# "-" : length 1
# "|" : length 2

# M = np.array([
#     [0, 8, 0, 0, 2,  0,  0, 0],
#     [8, 0, 1, 0, 0,  0,  0, 2],
#     [0, 1, 0, 5, 0,  2,  0, 0],
#     [0, 0, 5, 0, 0,  0,  2, 0],
#     [2, 0, 0, 0, 0,  12, 0, 0],
#     [0, 0, 2, 0, 12, 0,  5, 0],
#     [0, 0, 0, 2, 0,  5,  0, 0],
#     [0, 2, 0, 0, 0,  0,  0, 0]
# ]).T  # 各ノードからいけるノードを表す。
# example
# M[:, 1] : [8 0 1 0 0 0 0 2]
# これは node 2 からいける nodeである1,3,8に値を持つ行列
# 各値はnode間距離（上図参照）

# # calc setting
# start = 5  # nearest node : TODO : consider how to decide it
# goal = 8  # goal node : TODO : consider how to decide it
# P = gen_sequence(start, goal, M)
# print(P)  # show the result


def gen_sequence(start, goal, M):
    # initialize lists
    I = np.arange(M.shape[0]) + 1  # list of indices
    V = np.zeros_like(I)  # value list
    IV = np.vstack(
        (I, V)
    ).T  # valued index list : value is a minimum cost to reach the node
    ID = 0  # id
    GEN = 1  # generation = tree depth
    PN = 2  # parent number in generation
    NID = 3  # node id
    VAL = 4  # value
    ai = start  # current position
    H = np.zeros((M.shape[0] ** 2, 5))  # history
    # H = [id, generation, parent number in generation, node index, value]
    H[0, :] = [1, 1, 0, ai, 0]  # initial
    for i in range(2, M.shape[0] + 1):  # generation loop
        # print(f"Generation : {i}")  # for debug
        ids = H[:, GEN] == (i - 1)  # previous gen ids (logical vector)
        v0 = H[ids, NID].astype(int) - 1  # previous vertices
        V0 = H[ids, VAL]  # previous vertices' value
        Vi = M[:, v0]  # value from v0 to next vertices
        for col in range(Vi.shape[1]):  # loop for each previous vertex
            # print(f"col : {col}")  # for debug
            lH = int(np.argwhere(H[:, 0] == 0)[0][0]) - 1  # number of history
            V = Vi[:, col]  # value from v0(col) to next generation vi
            vi = np.nonzero(V)[0]  # node connected from v0(col)
            ei = V != 0  # edge indices
            V[ei] = V[ei] + V0[col]  # value from start to vi
            TV = IV[:, 1]  # Minimum value to each node

            # subs. large value(=1000) to 0 to compare
            TV[TV == 0] = 1000
            V[V == 0] = 1000

            mini = np.where(TV > V)[0]  # index to be replaced the value
            IV[mini, 1] = V[mini]  # set/update value
            ids = np.arange(lH + 1, lH + len(mini) + 1)  # indices for set/update nodes
            H[ids, :] = np.column_stack(
                [
                    ids + 1,
                    np.full(len(ids), i),
                    np.full(len(ids), col + 1),
                    mini + 1,
                    V[mini],
                ]
            )  # log history
        ngen = np.count_nonzero(H[:, GEN] == i)  # number of this generation
        if ngen == 0:  # break if there is no new node
            break
    # Find path from start to goal
    nid = goal  # node index
    id = H[:, NID] == nid  # id = row in H
    # print(f"{H}, {GEN}, {nid}, {NID}")
    gen = int(H[id, GEN][0])  # generation to reach the goal with minimum path
    P = np.zeros(gen, dtype=int)  # path array
    P[-1] = goal  # set goal
    for g in range(gen - 1, 0, -1):  # from goal to start
        gid = H[:, GEN] == g  # indices of g-th generation
        tmp = H[gid, :]  # g-th gen history
        tmp = tmp[int(H[id, PN][0]) - 1, :]  # find the parent of current node
        nid = int(tmp[NID])  # parent's node index
        id = H[:, NID] == nid  # parent's id in H
        P[g - 1] = nid  # set parent as a node on the path

    return P
