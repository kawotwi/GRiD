#!/usr/bin/python3
from URDFParser import URDFParser
from RBDReference import RBDReference
from util import parseInputs, printUsage, validateRobot, initializeValues, printErr
import numpy as np
import copy

def finiteDiff(q, diff_func, eps = 1e-3):
    pos = diff_func(q)
    npos = len(pos)
    nq = len(q)
    grads = np.zeros((npos,nq))
    for i in range(nq):
        q_p = copy.deepcopy(q)
        q_m = copy.deepcopy(q)
        q_p[i] += eps
        q_m[i] -= eps
        pos_p = diff_func(q_p)
        pos_m = diff_func(q_m)
        delta = pos_p - pos_m
        delta /= (2*eps)
        grads[:,i] = np.squeeze(delta)
    return grads

def main():
    URDF_PATH, DEBUG_MODE, _ = parseInputs()

    parser = URDFParser()
    robot = parser.parse(URDF_PATH)

    validateRobot(robot)

    reference = RBDReference(robot)
    q, qd, u, n = initializeValues(robot)

    grads = reference.end_effector_position_gradients(q)
    test_grads = finiteDiff(q,lambda a : reference.end_effector_positions(a)[0])
    print("Finite diff")
    print(test_grads)
    print("Analytical")
    print(grads)

if __name__ == "__main__":
    main()