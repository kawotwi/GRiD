#!/usr/bin/python3
from .URDFParser import URDFParser
from .GRiDCodeGenerator import GRiDCodeGenerator
from .util import parseInputs, printUsage, validateRobot

def main():
    URDF_PATH, DEBUG_MODE, FLOATING_BASE = parseInputs()

    parser = URDFParser()
    robot = parser.parse(URDF_PATH, floating_base = FLOATING_BASE)

    validateRobot(robot)

    codegen = GRiDCodeGenerator(robot,DEBUG_MODE,True)
    codegen.gen_all_code()
    print("New code generated and saved to grid.cuh!")

if __name__ == "__main__":
    main()