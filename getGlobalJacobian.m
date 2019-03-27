function [J,Jt,Jdot] = getGlobalJacobian(Jhip_h, JstFoot_h, JswFoot_h, Jtorso_h,
                                         Jhip_e, JstFoot_e, JswFoot_e, Jtorso_e,
                             Jhipdot_h, JstFootdot_h, JswFootdot_h, Jtorsodot_h,
                             Jhipdot_e, JstFootdot_e, JswFootdot_e, Jtorsodot_e)
  # This function creates the Jacobian for the global system from the Jacobians
  # of the human and of the exoskeleton. It also creates the transpose and 
  # the time derivative of the Jacobian.
  # 
  # input:  Jhip_h    - Jacobian for hip          \
  #         JstFoot_h - Jacobian for stance foot   \   for human
  #         JstFoot_h - Jacobian for swing foot    /
  #         Jtorso_h  - Jacobian for torso strap  /
  #         Jhip_e    - Jacobian for hip          \
  #         JstFoot_e - Jacobian for stance foot   \   for exoskeleton
  #         JstFoot_e - Jacobian for swing foot    /
  #         Jtorso_e  - Jacobian for torso strap  /
  #       <remaining> - time derivatives of these Jacobians
  #
  # output: J    - global Jacobian
  #         Jt   - transpose of global Jacobian
  #         Jdot - time derivative of global Jacobian
  
  Zero = zeros(size(JstFoot_e,1),size(Jhip_h,2));
  
  J = [-Jtorso_h,   Jtorso_e;
       -Jhip_h,     Jhip_e;
       -JstFoot_h,  JstFoot_e;
       -JswFoot_h,  JswFoot_e;
        Zero,       JstFoot_e];
        
  Jt = transpose(J);
  
  Jdot = [-Jtorsodot_h,   Jtorsodot_e;
          -Jhipdot_h,     Jhipdot_e;
          -JstFootdot_h,  JstFootdot_e;
          -JswFootdot_h,  JswFootdot_e;
           Zero,          JstFootdot_e];
           
##  # <debug>
##  J = [-Jtorso_h,   Jtorso_e;
##       -Jhip_h,     Jhip_e;
##       -JstFoot_h,  Zero;
##       -JswFoot_h,  JswFoot_e;
##        Zero,       JstFoot_e];
##        
##  Jt = transpose(J);
##  
##  Jdot = [-Jtorsodot_h,   Jtorsodot_e;
##          -Jhipdot_h,     Jhipdot_e;
##          -JstFootdot_h,  Zero;
##          -JswFootdot_h,  JswFootdot_e;
##           Zero,          JstFootdot_e];
  
endfunction
