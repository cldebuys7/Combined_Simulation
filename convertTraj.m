%function [x] = convertTraj(q)
  # This script converts a previous optimal trajectory for a biped into a
  # starting feasible trajectory for the biped and exoskeleton. It requires
  # the optimal trajectories from a pervious biped simulation (sadly) which is
  # stored in prev_space.m.
  #
  # input:  previous optimal trajectories (positions and velocities) for biped 
  #         in absolute coordinates with positive vertical reference
  #
  # output: starting desired trajectory for:
  #      1. biped: absolute coord with normal convention (start: hip)
  #      2. exo:   absolute coord with normal convention (start: hip)
  
  % script debug %
  %q = [q', dq']
  clear all; clc;
  addpath(genpath('Mathematica'))
  pkg load optim
  load('prev_space')
  %Determine time span. The gait cycle is cut short to only one step
  temp = linspace(0,1.23,50);
  tspan = temp(1:25);
  % end script debug %
  
  # biped joint positions
  x(:,3) = q(:,3) + pi/2;
  x(:,4) = q(:,2) - pi/2;
  x(:,5) = q(:,1) - pi/2;
  x(:,6) = q(:,4) - pi/2;
  x(:,7) = q(:,5) - pi/2;
  # biped hip position (stance foot as origin)
  for i = 1:size(q,1)
    pos_h = qEnds_h([0,0,x(i,3:7)]);
    # hipPos is -stFootPos if stFoot is taken as the origin
    x(i,1:2) = -pos_h(3,1:2);
    # again, subtract off stFootPos to make stFoot the reference
    SwAnkPos(i,1:2) = pos_h(5,1:2) - pos_h(3,1:2);
  endfor
  # biped joint velocities
  x(:,17) = q(:,8);
  x(:,18) = q(:,7);
  x(:,19) = q(:,6);
  x(:,20) = q(:,9);
  x(:,21) = q(:,10);
  # biped hip velocity (stance foot as origin)
  for i = 1:size(q,1)
    vel_h = vEnds_h([0,0,x(i,3:7),0,0,x(i,16:20)]);
    # hipVel is -stFootVel since stFoot is stationary
    x(i,15:16) = -vel_h(3,1:2);
    # again, subtract off stFootVel to make stFoot the reference
    SwAnkVel(i,1:2) = vel_h(5,1:2) - vel_h(3,1:2);
  endfor
  
  # exoskeleton positions
  HipPos = x(:,1:2);  
  q_guess = initialguess(tspan,SwAnkPos,SwAnkVel,HipPos);
  x(:,[8:9,11:14,22:23,25:28]) = q_guess;
  x(:,[10,24]) = x(:,[3,17]); # exo torso pos and vel are same as human
  
  # store x as init
  init = x;
  clear x
  save("init_space")
%endfunction