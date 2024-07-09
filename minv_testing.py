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
        print(f"q: {q}\n _q: {_q}\n  inds_q: {inds_q} ")
        Xmat = robot.get_Xmat_Func_by_id(curr_id)(_q)
        print('X[{}]\n{}'.format(curr_id, Xmat))
        # review minv robot. calls
        S = robot.get_S_by_id(curr_id)
        subtreeInds = robot.get_subtree_by_id(curr_id)
        parent_ind = robot.get_parent_id(curr_id)
        print(f'{2*'#'} [ID: {curr_id}, ParentInd: {parent_ind}] {2*'#'}\nS:\n{S}')

    # print(f'Imats Dict: {robot.get_Imats_dict_by_id()}')
    # print(f'Subtree ind type: {type(subtreeInds)}')
    # q[3:7] *= -1
    (Gc, Gv, Ga, Gf) = reference.rnea(q, qd)
    print('c\n{}\nv\n{}\na\n{}\nf\n{}'.format(Gc, Gv, Ga, Gf))
    print(f'NB: {NB}, n: {len(q)}')


    # Pinocchio model creation 
    if FLOATING_BASE:
        print('PINOCCHIO FLOATING BASE MODEL')
        #create new model from scratch
        """
        Using Pinocchio to create a new model from scratch use the following: 
            * pin.RobotWrapper.BuildFromURDF(URDF_PATH, root_joint = XXX )
            * replace XXX with pin.JointModelFreeFlyer(), JointModelSpherical(), or JointModelSphericalZYX
        Freeflyer q and v convention:
            * q = [global_pos_x, global_pos_y, global_pos_z, qX, qY, qZ, qW, joint angles:] (freeflyer, q = quaternion) 
            * qd = [6DoF joint, joint velocities] efectively +6
             * v = [x_trans, y_trans, z_trans, x_rot, y_rot, z_rot] (freeflyer)
            * v = [z_rot, y_rot, x_rot] (spherical or sphericalZYX) 
        """
        robot_p = pin.RobotWrapper.BuildFromURDF(URDF_PATH,root_joint = pin.JointModelFreeFlyer())
        fb_model = robot_p.model
        # fb_data = fb_model.createData()
        # j0 = pin.JointModelFreeFlyer() #optionally use JointModelSpherical() or JointModelSphericalZYX
        # build same model with root joint as free flyer
        # cycle quaternion from wxyz (GRiD) to xyzw (for pinocchio)
        fb_q = q.copy()
        fb_q[3], fb_q[4], fb_q[5], fb_q[6] = fb_q[4], fb_q[5], fb_q[6], fb_q[3]
        fb_q[3:7] *= -1
        print(fb_model)

        print('q: ',q)
        print('qd: ',qd)
        # fb_q[3:7] *= -1

        v_p = zero(robot_p.nv)
        print(f'Original q: {q}\n New q: {fb_q}')
        a_p = zero(robot_p.nv) #floating base gets 13 vals
        print(pin.rnea(robot_p.model,robot_p.data,fb_q,v_p,a_p))
    else:
        print('NO FLOATING BASE')
        robot_p = pin.RobotWrapper.BuildFromURDF(URDF_PATH)
        print(robot_p.model)
        a_p = zero(robot_p.nv) #floating base gets 13 vals
        fb_q = q
        

        print('q_p\n{}\nv_p\n{}\na_p{}'.format(q, qd, a_p))

    print('RNEA Results')
    tau_p = pin.rnea(robot_p.model, robot_p.data, fb_q, qd, a_p)
    val_comparison(Gc, tau_p,calc_diff=True)

    print('Pinocchio frame placements wrt origin')
    pin.forwardKinematics(robot_p.model,robot_p.data, fb_q)
    pin.neutral(robot_p.model)
    pin.updateFramePlacements(robot_p.model,robot_p.data)
    pin.computeAllTerms(robot_p.model,robot_p.data,np.array(fb_q),qd)
    if FLOATING_BASE:
        print(f"Original q: {q}\n quaternion q (w,x,y,z): {q[3:7]}\npinocchio q (x,y,z,w): {fb_q[3:7]}")
    # for name, oMi in zip(robot_p.model.names, robot_p.data.oMi):
    #     print(("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat)))
    #     print(f"{pin.Quaternion(oMi.rotation)}")
    #     print(f"{oMi.rotation}")
    # iterate through each frame and print linear and angular velocity
    # print transformation matrices for each joint pinocchio

    # print(f'{10 * ' # '}XMAT COMPARISON{10 * ' # '}')
    # print(q)
    # for curr_id in range(NB):
    #     inds_q = robot.get_joint_index_q(curr_id)
    #     _q = q[inds_q]
    #     print(f"q: {q}\n _q: {_q}\n  inds_q: {inds_q} ")
    #     Xmat = robot.get_Xmat_Func_by_id(curr_id)(_q)
    #     print('X[{}]\n{}'.format(curr_id, Xmat[:3,:3]))
    #     print(f"Frame {curr_id+2} transformation matrix:")
    #     print(robot_p.data.oMi[curr_id+1].homogeneous[:3,:3])
        
    #     print(f'DIFFERENCE:\n {Xmat[:3,:3] - robot_p.data.oMi[curr_id+1].homogeneous[:3,:3]}')
    # COMMENT EVERYTHNIG OUT BELOW THIS LINE

    # # print(f'tau: {tau_p}')
    # # GRid homogenous matrix placements
    
    # # for i in range(19):
    # #     print('Frame %s:' % (i+1))
    # #     print(robot_p.data.oMf[i])

    # """ Uncomment below in order to test fixed values from matlab with GRiD
    # # matlab_minv_fixed = [[0.6909,    0.2027,   -1.0078,   -0.3443,    1.2005,   -0.5809,  -0.3708],
    # #                     [0.2027,    0.6237,  -0.7856,    0.3962,    0.8743,   -0.6563,   -0.1582],
    # #                     [-1.0078,   -0.7856,    3.1658,    0.2670,   -0.7784,    4.4264,   -1.1277],
    # #                     [-0.3443,    0.3962,    0.2670,    1.9373,    0.5185,    3.5768,   -0.4423],
    # #                     [1.2005,    0.8743,   -0.7784,    0.5185,   83.8785,    3.1718,  -81.5563],
    # #                     [-0.5809,   -0.6563,    4.4264,    3.5768,    3.1718,  63.6370,   -4.6936],
    # #                     [-0.3708,   -0.1582,   -1.1277,   -0.4423,  -81.5563,   -4.6936,  280.7079]]
    # """
    # # # Minv calculations
    # grid_minv = reference.test_minv(q)
    # # print(f'GRiD Minv ({grid_minv.shape}):\n{grid_minv}')
    # pin_minv = pin.computeMinverse(robot_p.model,robot_p.data,fb_q) # TODO double check accuracy
    # # print(f'Pin Minv ({pin_minv.shape}) ({pin_minv.shape}):\n{pin_minv}')
    # print("Minv Comparison: GRiD (1) vs. Pinocchio (2)")
    # val_comparison(grid_minv, pin_minv,calc_diff=False)

    # # pinocchio crba inverse
    # pin_crba_h = pin.crba(robot_p.model,robot_p.data,q)
    # # print(f"{10 * '#'}\n PIN CRBA: \n{pin_crba_h}\n PIN Inverse: {np.linalg.inv(pin_crba_h)}")

    # # GRiD crba
    # grid_crba = reference.crba(q,qd)
    # # print(f"{10 * '#'}\n GRID CRBA: \n{grid_crba}")
    # # print(f"{10 * '#'}\n GRID CRBA INVERSE: \n{np.linalg.inv(grid_crba)}")

    # print("CRBA Comparison: GRiD (1) vs. Pinocchio (2)")
    # val_comparison(grid_crba, pin_crba_h,calc_diff=True)
    # print("CRBA Inverse Comparison: GRiD (1) vs. Pinocchio (2)")
    # val_comparison(np.linalg.inv(grid_crba), np.linalg.inv(pin_crba_h),calc_diff=True)


def val_comparison(val1, val2,calc_diff=False):
    """Simple val comparison function for matrices of the same shape."""
    print(f"{10*'#'}\n Method 1 ({val1.shape}):\n {val1}")
    print(f"{10*'#'}\n Method 2 ({val2.shape}):\n {val2}")
    if calc_diff: print(f"{10*'#'}\n Difference:\n {val1-val2}")

if __name__ == "__main__":
    main()