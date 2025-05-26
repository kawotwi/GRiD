# This file takes a urdf file and generates the correct grid.cuh file. Run before setup.py if no grid.cuh file included.

import os 
import sys
import shutil
from pathlib import Path

# import from parent directory 
original_sys_path = sys.path.copy()
parent_dir = Path.cwd().parent
sys.path.insert(0,str(parent_dir))

from URDFParser import URDFParser
from GRiDCodeGenerator import GRiDCodeGenerator
from util import parseInputs, printUsage, validateRobot

# go back to original path
sys.path = original_sys_path

def main():
    URDF_PATH,DEBUG_MODE, FILE_NAMESPACE_NAME, FLOATING_BASE = parseInputs()
    parser = URDFParser()
    
    robot = parser.parse(URDF_PATH, floating_base = FLOATING_BASE)
    validateRobot(robot)

    codegen = GRiDCodeGenerator(robot,DEBUG_MODE,True, FILE_NAMESPACE = FILE_NAMESPACE_NAME)
    codegen.gen_all_code(include_homogenous_transforms = True) # see commit history if there are issues with this or above line.
    print("New code generated and saved to grid.cuh!")
    
    shutil.move("grid.cuh", "include/grid.cuh")

if __name__ == "__main__":
    main()
