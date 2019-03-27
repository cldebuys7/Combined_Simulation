# This script generates an optimal trajectory for the human-exoskeleton system
#  using single shooting.

# !!! try squatting
clear; clc; close all;
addpath(genpath('Mathematica'))
pkg load optim
# pkg load odepkg

# load workspace with desired trajectories (init) and parameters (p)
  load('init_space')

# parameters
  p = param;
# time vector
  t = tspan;
# desired trajectories
  des = init; # each row is one time instance
  order = 6;
  [d_coeff] = getDesiredCoeff(t,des,order); # each row is a polynomial's coeff
  tFinal = p.stepTime;
  [d] = getDesiredPoly(t,d_coeff,tFinal); # each column is one time instance
# initial condition for entire system
  x0 = [des(1,:) 0 0 0 0]'; # !! may need to get d(1,:)
# draw the initial condition
  drawConfig(t,x0)
# initial guess (coefficients of 4 polynomials, stacked into a vector)
  m = size(d_coeff,1);
  n = size(d_coeff,2);
  for i = 1:m
    Cd0(1+(i-1)*n:n+(i-1)*n) = [d_coeff(i,:)];
  endfor
  
################################################################################
# a global indicator of the optimization process. Meant for debugging.
  global y
  y = 0
################################################################################

# run trajectory optimization (single shooting)
[Cd,obj,info,iter,nf,lambda] = sqp(Cd0, @(Cd)objfun(Cd, t, p, d, des, x0, m, n),
                                                   [],[],[],[],50)
               # !!! might need to add constraints or bounds to the opt