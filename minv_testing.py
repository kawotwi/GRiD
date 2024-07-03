# minv_testing.py: a benchmark testing file comparing pinocchio and grid
# GRiD Imports
from RBDReference.RBDReference import RBDReference # depending on system .RBDReference is unnecessary
from URDFParser.URDFParser import URDFParser # depending on system .URDFParser is unnecessary 
# from GRiDCodeGenerator import GRiDCodeGenerator
from util import parseInputs, printUsage, validateRobot, initializeValues, printErr
import numpy as np
# Piniocchio Imports
import pinocchio as pin
from pinocchio.utils import * 
import inspect

def main():
    URDF_PATH, DEBUG_MODE, FLOATING_BASE = parseInputs()
    parser = URDFParser()
    robot = parser.parse(URDF_PATH, floating_base = FLOATING_BASE)

    validateRobot(robot)

    reference = RBDReference(robot)
    q, qd, u, n = initializeValues(robot, MATCH_CPP_RANDOM = True)
    print(q)
    print(qd)
    # Zero velocity and position variables for singularity testingw
    qd = 0 * qd

    print('q\n{}\nqd\n{}\nu'.format(q, qd, u))

    NB = robot.get_num_bodies()
    for curr_id in range(NB):
        inds_q = robot.get_joint_index_q(curr_id)
        _q = q[inds_q]
        Xmat = robot.get_Xmat_Func_by_id(curr_id)(_q)
        print('X[{}]\n{}'.format(curr_id, Xmat))
        # review minv robot. calls
        S = robot.get_S_by_id(curr_id)
        subtreeInds = robot.get_subtree_by_id(curr_id)
        parent_ind = robot.get_parent_id(curr_id)
        print(f'{2*'#'} [ID: {curr_id}, ParentInd: {parent_ind}] {2*'#'}\nS:\n{S}')

    # print(f'Imats Dict: {robot.get_Imats_dict_by_id()}')
    # print(f'Subtree ind type: {type(subtreeInds)}')
    # (Gc, Gv, Ga, Gf) = reference.rnea(q, qd)
    # print('c\n{}\nv\n{}\na\n{}\nf\n{}'.format(Gc, Gv, Ga, Gf))
    print(f'NB: {NB}, n: {len(q)}')


    # Pinocchio model creation 
    if FLOATING_BASE:
        print('FLOATING BASE')
        #create new model from scratch
        robot_p = pin.RobotWrapper.BuildFromURDF(URDF_PATH,root_joint = pin.JointModelFreeFlyer())
        fb_model = robot_p.model
        # fb_data = fb_model.createData()
        # j0 = pin.JointModelFreeFlyer() #optionally use JointModelSpherical() or JointModelSphericalZYX
        # build same model with root joint as free flyer

        #fb q generation
        print("q = {[", end="")
        # Getting the floating base position vals
        fb_qx = [0] * 7
        # Get the quaternion
        fb_qx[:4] = q[3:7]
        # Cycle the quaternion from wxyz (GRiD) to xyzw (for pinocchio)
        w = fb_qx[0]
        fb_qx[:3] = fb_qx[1:4]
        fb_qx[3] = w
        # Get the xyz for the floating base positioning
        fb_qx[4:] = q[:3]

        print("; ".join(str(x) for x in fb_qx) + "]; ", end = "")
        print("; ".join(str(x) for x in q[7:]), end = "}\n")
        fb_q = fb_qx
        print(fb_q)
        print(fb_model)

        print('q: ',q)
        print('qd: ',qd)
        fb_q = [fb_qx[0],fb_qx[1],fb_qx[2],fb_qx[3],fb_qx[4],fb_qx[5],fb_qx[6],q[7],q[8],q[9],q[10],q[11],q[12],q[13]]
        print(f'Original q: {q}\n New q: {fb_q}')
        a_p = zero(13) #floating base gets 13 vals
    else:
        print('NO FLOATING BASE')
        robot_p = pin.RobotWrapper.BuildFromURDF(URDF_PATH)
        print(robot_p.model)
        a_p = zero(robot_p.nv) #floating base gets 13 vals

        print('q_p\n{}\nv_p\n{}\na_p{}'.format(q, qd, a_p))
    # print(inspect.signature(pin.RobotWrapper.BuildFromURDF))


    """ Uncomment below in order to test fixed values from matlab with GRiD
    # matlab_minv_fixed = [[0.6909,    0.2027,   -1.0078,   -0.3443,    1.2005,   -0.5809,  -0.3708],
    #                     [0.2027,    0.6237,  -0.7856,    0.3962,    0.8743,   -0.6563,   -0.1582],
    #                     [-1.0078,   -0.7856,    3.1658,    0.2670,   -0.7784,    4.4264,   -1.1277],
    #                     [-0.3443,    0.3962,    0.2670,    1.9373,    0.5185,    3.5768,   -0.4423],
    #                     [1.2005,    0.8743,   -0.7784,    0.5185,   83.8785,    3.1718,  -81.5563],
    #                     [-0.5809,   -0.6563,    4.4264,    3.5768,    3.1718,  63.6370,   -4.6936],
    #                     [-0.3708,   -0.1582,   -1.1277,   -0.4423,  -81.5563,   -4.6936,  280.7079]]
    """
    # # Minv calculations
    grid_minv = reference.test_minv(q)
    # print(f'GRiD Minv ({grid_minv.shape}):\n{grid_minv}')
    pin_minv = pin.computeMinverse(robot_p.model,robot_p.data,q) # TODO double check accuracy
    # print(f'Pin Minv ({pin_minv.shape}) ({pin_minv.shape}):\n{pin_minv}')
    val_comparison(grid_minv, pin_minv,calc_diff=False)

    # pinocchio crba inverse
    pin_crba_h = pin.crba(robot_p.model,robot_p.data,q)
    print(f"{10 * '#'}\n CRBA: \n{pin_crba_h}\n Inverse: {np.linalg.inv(pin_crba_h)}")

    # GRiD crba
    grid_crba = reference.crba(q,qd)
    print(f"{10 * '#'}\n CRBA: \n{grid_crba}")

def val_comparison(val1, val2,calc_diff=False):
    """Simple val comparison function for matrices of the same shape."""
    print(f"{10*'#'}\n Method 1 ({val1.shape}):\n {val1}")
    print(f"{10*'#'}\n Method 2 ({val2.shape}):\n {val2}")
    if calc_diff: print(f"{10*'#'}\n Difference:\n {val1-val2}")

if __name__ == "__main__":
    main()