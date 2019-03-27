function dx = dynamics(t,x,p,d_coeff,m,op)
  # This function is called by an ode to solve the system dynamics for both the
  # human and the exoskeleton in order to obtain the optimal trajectories.
  #
  # input:  t - time
  #         x - state vector
  #         u - control input chosen by optimizer
  #         p - struct of physical parameters
  #   d_coeff - desired trajectories (polynomial coefficients)
  #        op - option flag (op = 2 includes F_contr in the state)
  #
  # output: dx - time derivative of state vector
  #
  # State vector:
    # x(1:7)    - positions of human (phx,phy,qh1,qh2,qh3,qh4,qh5)
    #             x( 1) = phx -> hip horizontal position
    #             x( 2) = phy -> hip vertical position
    #             x( 3) = qh1 -> torso angle
    #             x( 4) = qh2 -> stance thigh angle
    #             x( 5) = qh3 -> stance shank angle
    #             x( 6) = qh4 -> swing thigh angle
    #             x( 7) = qh5 -> swing shank angle
    #
    # x(8:14)   - positions of exoskeleton (pex,pey,qe1,qe2,qe3,qe4)
    #             x( 8) = pex -> hip horizontal position
    #             x( 9) = pey -> hip vertical position
    #             x(10) = qh1 -> torso angle
    #             x(11) = qe1 -> stance thigh angle
    #             x(12) = qe2 -> stance linear actuator displacement
    #             x(13) = qe3 -> swing thigh angle
    #             x(14) = qe4 -> swing linear actuator displacement
    #
    # x(15:21)  - velocities of human
    # x(22:28)  - velocities of exoskeleton
  
  # Assign state vector to some easier to read variables 
    [qh, qe, dqh, dqe] = getState(x);
    q  = [qh; qe];
    dq = [dqh; dqe];
    xe = [qe; dqe];

  # Define human system matrices and Jacobians
    m = m/3; # number of control inputs
    [M_h, C_h, G_h, B_h] = getSystemMatrices(qh,dqh,m,1);   
    [Jhip_h,   JstFoot_h,   JswFoot_h,   Jtorso_h ...
     Jhipdot_h,JstFootdot_h,JswFootdot_h,Jtorsodot_h] = getJacobians(qh,dqh,1);
    
  # Define exoskeleton system matrices and Jacobians
    [M_e, C_e, G_e, B_e] = getSystemMatrices(qe,dqe,m,2);
    [Jhip_e,   JstFoot_e,   JswFoot_e,   Jtorso_e ...
     Jhipdot_e,JstFootdot_e,JswFootdot_e,Jtorsodot_e] = getJacobians(qe,dqe,2);
  
  # Define global (combined) system
    [M,C,G,B] = getGlobalSystemMatrices(M_h,C_h,G_h,B_h, M_e,C_e,G_e,B_e);
    # constraining forces
    # F = [F_torso; F_hip; F_stFoot; F_swFoot; GRF] (9x1)
    [J,Jt,Jdot] = getGlobalJacobian(Jhip_h, JstFoot_h, JswFoot_h, Jtorso_h,
                                    Jhip_e, JstFoot_e, JswFoot_e, Jtorso_e,
                            Jhipdot_h, JstFootdot_h, JswFootdot_h, Jtorsodot_h,
                            Jhipdot_e, JstFootdot_e, JswFootdot_e, Jtorsodot_e);
    
  # Calculate the dynamics (dx)
    H = C*dq + G;
    JMinvJt = J/M*Jt;
    cj = cond(JMinvJt)
    n = size(xe); # length of position vector
    # rearrange to the form dx = f(x) + g(x)u
    f = [dq; M \ ( - H + Jt/( JMinvJt ) * ( J/M*H - Jdot*dq ) )];
    g = [zeros(n,m); M \ ( eye(length(q)) - Jt/( JMinvJt )*(J/M) )*B];
    # calculate the control input (u) using feedback linearization
    u = getControlInput (t, xe, p, d_coeff, f, g);
    # calculate the dynamics
    dx = f + g*u;
    # <debug> check if constraint is satisfied
    ddq = dx(15:28);
    ddh = J*ddq + Jdot*dq;  # !!should be 0
    dh  = J*dq;             # !!should be 0
    # include u in dx (integral is taken, so take derivative outside)
    dx(29:32) = u;
    # switch 'op' is used to decide if F is included in the state
    switch op
      case 1
        # do nothing
      case 2
        # calculate F and add it to dx (take derivative outside)
        # F = [T_torso; F_hip; F_stFoot; F_swFoot; GRF] (9x1)
        F = ( JMinvJt ) \ ( J/M*(H - B*u) - Jdot*dq );
        dx(33:41) = F;
    endswitch
endfunction