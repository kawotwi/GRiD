import numpy as np
import gridCuda

def main():
    print("Testing gridCuda bindings....")

    # create instance of module with default gravity 
    grid = gridCuda.GRidDataFloat()

    # print some constants
    print(f"Number of joints: {gridCuda.NUM_JOINTS}")

    # generate random joint positions, velocities, and controls
    np.random.seed(0)
    q = np.random.normal(0, 1, gridCuda.NUM_JOINTS).astype(np.float32)
    qd = np.random.normal(0, 1, gridCuda.NUM_JOINTS).astype(np.float32)
    u = np.random.normal(0, 1, gridCuda.NUM_JOINTS).astype(np.float32)

    # set the state
    grid.load_joint_info(q, qd, u)

    # print the inputs
    print("\nJoint positions (q):")
    print(q)
    
    print("\nJoint velocities (qd):")
    print(qd)
    
    print("\Torque inputs (u):")
    print(u)

    # compute inverse dynamics 
    c = grid.inverse_dynamics()
    print("\nInverse dynamics (c):")
    print(c)

if __name__ == "__main__":
    main()
