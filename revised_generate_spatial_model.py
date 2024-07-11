#!/usr/bin/python3

from URDFParser.URDFParser import URDFParser
from util import parseInputs, printUsage, validateRobot
from numpy import identity, zeros
import sys

DOF1_J_TYPE = ["'Rx'", "'Ry'", "'Rz'", "'Px'", "'Py'", "'Pz'"]

"""
Matlab Model Generator
Parses NB, parent array, jtype array, Xtree and I matrices from URDF 
and create .m file compatible with Featherstone source code in working directory

Modified by: Naren Loganathan 
Updated to handle floating base model representation
Requires the URDFParser (doesn't call it with floating base no matter what)
Reference: http://royfeatherstone.org/spatial/v2/sysmodel.html
"""

# Helper function to add a 6x6 parameter to the robot defn
def add_6x6_param(name, index, mat):
    print(name + "{" + str(index) + "} = [", end = "")
    for i in range(6):
        for j in range(6):
            if j == 0 and i != 0:
                print("\t\t", end = "")
            if j > 0:
                print(" ", end = "")
            print(mat[i, j], end = "")
            if j == 5:
                print(";" if i != 5 else "];\n")

def generate_matlab_model(robot, floating_base):
    
    original_stdout = sys.stdout
    f = open(f"{robot.name}.m",  "w")
    sys.stdout = f
    print(f"function robot = {robot.name}()")

    # robot.NB is the number of joints, or the number of links excluding the fixed base

    # w/o floating base: just the number of joints in `robot`

    # w/ floating base: number of links (including the original fixed base)

    # which is the number of original joints + 1, effectively we're doing:
    # [fixed base] Fb [original fixed base]

    # robot.nb
    if floating_base:
        print(f"\n\trobot.NB = {robot.get_num_joints() + 1};")
    else:
        print(f"\n\trobot.NB = {robot.get_num_joints()};")
    
    # robot.parent
    parent_array = robot.get_parent_id_array()
    if floating_base:
        # one new link, and we include the original fixed base as link number 1, with its parent being 0
        parent_array = [0] + [parent + 1 + 1 for parent in parent_array]
    else:
        # Since the fixed base link is -1, and so on...
        for i in range(len(parent_array)):
            parent_array[i] = parent_array[i] + 1

    print(f"\trobot.parent = [", end="")
    print(*parent_array, "];")
    
    # robot.jtype
    if floating_base:
        print("\trobot.jtype = {'Fb', ", end="")
    else:
        print("\trobot.jtype = {", end="")

    joints = robot.get_joints_ordered_by_id()
    jtype_array = []
    for i in range(robot.get_num_joints()):
        s = joints[i].S 
        for index in range(len(s)):
            if s[index] == 1:
                jtype_array.append(DOF1_J_TYPE[index])
                if i != robot.get_num_joints() - 1:
                    jtype_array.append(", ")
                break
    print(''.join(jtype_array) + "};\n")

    # robot.Xtree and robot.I
    for x in range(len(joints)):
        add_6x6_param("\trobot.Xtree", x + (2 if floating_base else 1), joints[x].origin.Xmat_sp_fixed)
        add_6x6_param("\trobot.I", x + (2 if floating_base else 1), robot.get_Imat_by_id(x))
        
    if floating_base:
        # the preceding Fb joint has an identity transformation matrix
        print("\trobot.Xtree{1} = eye(6);")

        # the original fixed base might have mass, that becomes relevant here
        # note that GRiD compresses fixed joints
        # inertia from the original fixed joint child link can get 'pushed upwards'
        add_6x6_param("\n\trobot.I", 1, robot.get_Imat_by_id(-1))
        
    sys.stdout = original_stdout
    f.close()

def main():
    URDF_PATH, DEBUG_MODE, FLOATING_BASE = parseInputs()

    parser = URDFParser()
    robot = parser.parse(URDF_PATH)

    validateRobot(robot)

    # Creates the Matlab model
    generate_matlab_model(robot, FLOATING_BASE)
    print(f"m file generated and saved to {robot.name}.m!")

if __name__ == "__main__":
    main()
