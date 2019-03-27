function [qh, qe, dqh, dqe] = getState (x)
  # This function partitions the state vector into the human and exoskeleton
  # position and velocity vectors.
  # 
  #
  # input:  x - state vector
  #
  # output: qh  - position vector of human
  #         qe  - position vector of exoskeleton
  #         dqh - velocity vector of human
  #         dqe - velocity vector of exoskeleton
    
  qh  = x( 1: 7);
  qe  = x( 8:14);
  dqh = x(15:21);
  dqe = x(22:28);
    
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
endfunction
