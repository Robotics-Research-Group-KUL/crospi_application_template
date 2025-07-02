import casadi
import numpy as np
import pdb

def wrench2numpy(wrench):
    return np.array([wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z])

def get_cog(wrenches:list, rotations:list):

    wrench_list_numpy = [wrench2numpy(wrench) for wrench in wrenches]
    print("Rotations: ",rotations)
    print("Wrenches: ",wrench_list_numpy)
    
    opti = casadi.Opti()

    P = opti.variable(3,1)
    wz = opti.variable()
    W = casadi.vertcat(0,0,-wz)

    opti.set_initial(wz, 0)
    opti.set_initial(P[0], 0)
    opti.set_initial(P[1], 0)
    opti.set_initial(P[2], 0.1)

    opti.subject_to( -100 <= wz )
    opti.subject_to( -1 <= P[0] )
    opti.subject_to( -1 <= P[1] )
    opti.subject_to( -1 <= P[2] )
    opti.subject_to( wz <= 100 )
    opti.subject_to( P[0] <= 1 )
    opti.subject_to( P[1] <= 1 )
    opti.subject_to( P[2] <= 1 )

    objective = 0
    # Measured wrenches in load cell for different configurations
    #TODO: Add check of minimum three.
    # TODO: Handle the frame change between sensors

    for idx, wrench in enumerate(wrench_list_numpy):
        F = opti.parameter(3,1)
        opti.set_value(F,wrench[0:3])
        M = opti.parameter(3,1)
        opti.set_value(M,wrench[3:6])
        R_T = opti.parameter(3,3)
        opti.set_value(R_T,np.transpose(rotations[idx]))
        exp_F = (R_T@W + W) - F
        exp_M = casadi.cross(P, R_T@W) + casadi.cross(P, W) - M
        objective += exp_F.T@exp_F + exp_M.T@exp_M

    opti.minimize(objective)
    opti.solver('ipopt')

    sol = opti.solve()
    
    return {"position":sol.value(P).tolist(), "weight":sol.value(wz)}

