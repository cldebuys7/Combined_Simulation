function F = exoInit(z,pos,vel)
  # This function calculates the exoskeleton joint angles which satisfy the
  # equality: human foot position = exoskeleton foot position
  #
  # input:    z - joint angles and velocities of exoskeleton (guess from fsolve)
  #         pos - joint angles of human
  #         vel - joint velocities of human
  #
  # output: F - fsolve tries to get F = 0
##  addpath(genpath('Mathematica'))

#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<#
# This code goes outside the function (if we ever get it working)
##  # exoskeleton
##  z0 = [3*pi/2, 0.1, 3*pi/2, 0.1, 0 ,0 ,0 ,0]';
##  lb = [pi/2,0,pi/2,0,-100,-100,-100,-100]';
##  ub = [2*pi,0.5,2*pi,0.5,100,100,100,100]';
##  for i = 1:size(q,1)
##    i
##    f = @(w) exoInit(w,x(i,3:7),x(i,14:18)); % func of dummy variable w
##    z = lsqnonlin(f,z0,lb,ub);
##    z0 = z;
####    lb = z - [pi/4,z(2),pi/4,z(4),100,100,100,100]';
####    lb'
####    ub = z + [pi/4,0.3,pi/4,0.3,100,100,100,100]';
##    
##    x(i,[8:11,19:22]) = z;
##  endfor
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>#
  x =  [0, 0, pos];
  dx = [0, 0, vel];
  w =  [0; 0; z(1:4)]';
  dw =  [0; 0; z(5:8)]';
  
  # Get positions
  qends_h =     qEnds_h(x);
  qends_e =     qEnds_e(w);
  
  stFootPos_h = qends_h(3,1:2);
  stFootPos_e = qends_e(3,1:2);
  
  swFootPos_h = qends_h(5,1:2);
  swFootPos_e = qends_e(5,1:2);
  
  # Get velocities
  vends_h =     vEnds_h([x dx]);
  vends_e =     vEnds_e([w dw]);
  
  stFootVel_h = vends_h(3,1:2);
  stFootVel_e = vends_e(3,1:2);
  
  swFootVel_h = vends_h(5,1:2);
  swFootVel_e = vends_e(5,1:2);
  
  # Drive difference to zero

  F(1:2) = stFootPos_h - stFootPos_e;
  F(3:4) = swFootPos_h - swFootPos_e;
  F(5:6) = stFootVel_h - stFootVel_e;
  F(7:8) = swFootVel_h - swFootVel_e;
  F;
endfunction