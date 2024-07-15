#!/usr/bin/python3
from URDFParser.URDFParser import URDFParser
from RBDReference.RBDReference import RBDReference
from GRiDCodeGenerator import GRiDCodeGenerator
from util import parseInputs, printUsage, validateRobot, initializeValues, printErr
import numpy as np

def main():
    URDF_PATH, DEBUG_MODE, FLOATING_BASE = parseInputs()

    parser = URDFParser()
    robot = parser.parse(URDF_PATH, floating_base = FLOATING_BASE)

    validateRobot(robot)

    reference = RBDReference(robot)
    q, qd, u, n = initializeValues(robot, MATCH_CPP_RANDOM = True)

    qd = 0*qd

    print("q")
    print(q)
    print("qd")
    print(qd)
    print("u")
    print(u)

    NB = robot.get_num_bodies()
    for curr_id in range(NB):
        inds_q = robot.get_joint_index_q(curr_id)
        _q = q[inds_q]
        Xmat = robot.get_Xmat_Func_by_id(curr_id)(_q)
        print("X[",curr_id,"]")
        print(Xmat)

    print("RNEA (Tau, Velocity, Acceleration, Force)")
    (c, v, a, f) = reference.rnea(q,qd)
    print("v")
    print(v)
    print("a")
    print(a)
    print("f")
    print(f)
    print("c")
    print(c)

    print("Minv")
    Minv = reference.minv(q)
    print(Minv)

    print("CRBA")
    M = reference.crba(q,qd)
    print(M)

    print("qdd")
    qdd = np.matmul(Minv,(u-c))
    print(qdd)

    # dc_du = reference.rnea_grad(q, qd, qdd)
    # print("dc/dq with qdd")
    # print(dc_du[:,:n])
    # print("dc/dqd with qdd")
    # print(dc_du[:,n:])

    # df_du = np.matmul(-Minv,dc_du)
    # print("df/dq")
    # print(df_du[:,:n])
    # print("df/dqd")
    # print(df_du[:,n:])

    if DEBUG_MODE:
        print("-------------------")
        print("printing intermediate outputs from refactorings")
        print("-------------------")
        codegen = GRiDCodeGenerator(robot, DEBUG_MODE)
        (c, v, a, f) = codegen.test_rnea(q,qd)
        print("v")
        print(v)
        print("a")
        print(a)
        print("f")
        print(f)
        print("c")
        print(c)
        
        Minv = codegen.test_minv(q)
        print("Minv")
        print(Minv)

        print("u-c")
        umc = u-c
        print(umc)
        print("qdd")
        qdd = np.matmul(Minv,umc)
        print(qdd)
        
        dc_du = codegen.test_rnea_grad(q, qd, qdd)
        print("dc/dq with qdd")
        print(dc_du[:,:n])
        print("dc/dqd with qdd")
        print(dc_du[:,n:])
        
        df_du = np.matmul(-Minv,dc_du)
        print("df/dq")
        print(df_du[:,:n])
        print("df/dqd")
        print(df_du[:,n:])

if __name__ == "__main__":
    main()