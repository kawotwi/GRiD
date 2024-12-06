#!/usr/bin/python3
from URDFParser.URDFParser import URDFParser
from RBDReference.RBDReference import RBDReference
from GRiDCodeGenerator import GRiDCodeGenerator
from util import parseInputs, printUsage, validateRobot, initializeValues, printErr
import numpy as np

def main():
    URDF_PATH, DEBUG_MODE, FLOATING_BASE, FILE_NAMESPACE_NAME = parseInputs()

    parser = URDFParser()
    robot = parser.parse(URDF_PATH, floating_base = FLOATING_BASE)

    validateRobot(robot)

    reference = RBDReference(robot)
    q, qd, u, n = initializeValues(robot, MATCH_CPP_RANDOM = True)

    u = 0*u

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
    print('Minv - inv(M): Testing CRBA vs. Minv (should be zero)')
    print(np.linalg.inv(M) - Minv)

    print("qdd")
    qdd = np.matmul(Minv,(u-c))
    print(qdd)

    print('aba')
    aba_qdd = reference.aba(q,qd,c, f_ext = [])
    print(aba_qdd)

    # NOTE qdd above does not match qdd below... why is that? Check same calculation in matlab of Minv @ (u-c)
    print("dc_dq") 
    # qdd = np.array([0.741788, 1.92844, -0.903882, 0.0333959, 1.17986, -1.94599, 0.32869, -0.139457, 2.00667, -0.519292, -0.711198, 0.376638, -0.209225])
    qdd = u
    dc_dq, dc_dqd = reference.rnea_grad(q, qd, qdd)
    print(dc_dq)
    print("dc_dqd")
    print(dc_dqd)

    dqdd_dq, dqdd_dqd, dqdd_dc = reference.forward_dynamics_grad(q,qd,c)
    print("dqdd_dq")
    print(dqdd_dq)
    print("dqdd_dqd")
    print(dqdd_dqd)
    print("dqdd_dc")
    print(dqdd_dc)

    # forward dynamics
    df_dq = np.matmul(-Minv,dc_dq)
    df_dqd = np.matmul(-Minv,dc_dqd)
    print(f"df/dq {df_dq.shape}")
    print(df_dq)
    print("df/dqd")
    print(df_dqd)

    external_forces = np.zeros((6,NB))
    qdd = reference.aba(q,qd,c, f_ext = [])


    if DEBUG_MODE:
        print("-------------------")
        print("printing intermediate outputs from refactorings")
        print("-------------------")
        codegen = GRiDCodeGenerator(robot, DEBUG_MODE, FILE_NAMESPACE = FILE_NAMESPACE_NAME)
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
