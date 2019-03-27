function [Jhip,   JstFoot,   JswFoot,   Jtorso ...
          Jhipdot,JstFootdot,JswFootdot,Jtorsodot] = getJacobians(q,dq,op)
  # This function creates the Jacobians.
  # 
  # input:  q  - position vector
  #         dq - velocity vector
  #         op - selects human (1) or exoskeleton (2)
  #
  # output: Jhip    - Jacobian for hip
  #         JstFoot - Jacobian for stance foot
  #         JstFoot - Jacobian for swing foot
  #         Jtorso  - Jacobian for torso strap location
  #     <remaining> - time derivatives of these Jacobians
  
  x = [q;dq]; # Jdot needs position and velocity information
  
  switch op
    case 1
    # define human Jacobians
    temp = Jhip_h(q);
    Jhip = temp(1:2,:);
    
    temp = JstFoot_h(q);
    JstFoot = temp(1:2,:);
    
    temp = JswFoot_h(q);
    JswFoot = temp(1:2,:);
    
    temp = Jtorso_h(q);
    Jtorso = temp(1,:);
    
    # define time derivative of Jacobian
    temp = Jhipdot_h(x);
    Jhipdot = temp(1:2,:);
    
    temp = JstFootdot_h(x);
    JstFootdot = temp(1:2,:);
    
    temp = JswFootdot_h(x);
    JswFootdot = temp(1:2,:);
    
    temp = Jtorsodot_h(x);
    Jtorsodot = temp(1,:);

    case 2
    # define exoskeleton Jacobians
    temp = Jhip_e(q);
    Jhip = temp(1:2,:);
    
    temp = JstFoot_e(q);
    JstFoot = temp(1:2,:);
    
    temp = JswFoot_e(q);
    JswFoot = temp(1:2,:);
    
    temp = Jtorso_e(q);
    Jtorso = temp(1,:);
    
    # define time derivative of Jacobian
    temp = Jhipdot_e(x);
    Jhipdot = temp(1:2,:);
    
    temp = JstFootdot_e(x);
    JstFootdot = temp(1:2,:);
    
    temp = JswFootdot_e(x);
    JswFootdot = temp(1:2,:);
    
    temp = Jtorsodot_e(x);
    Jtorsodot = temp(1,:);
    
    otherwise
      disp('Invalid input to getJacobians')
      
    endswitch

endfunction