 function cost = objfun (Cu, t, p, d, des, x0, m, n)
  # This function is called by sqp to obtain the objective value (cost) for a 
  # given quess 'z'. 'z' is used to solve the system dynamics for the system, 
  # and the cost is obtained from the weighted sum of: the square of the state 
  # error, the square of the final error, and the square of the control effort.
  #
  # input:  Cu - guess from sqp (coefficients of polynomials, stacked)
  #         t  - time vector
  #         p  - struct of physical parameters
  #         d  - desired trajectories (polynomial fits)
  #    d_coeff - desired trajectories (polynomial coefficients)
  #         x0 - initial position (determined from d)
  #         m  - number of rows of d_coeff
  #         n  - number of columns of d_coeff
  #
  # output: cost - weighted sum of the squares of the state error, the final 
  #                state error, the control effort

  
  # unstack the polynomial coefficients to get d_coeff (guess)
  for i = 1:m
    d_coeff(i,:) = Cu(1+(i-1)*n:n+(i-1)*n);
    # Cd0(1+(i-1)*n:n+(i-1)*n) = [d_coeff(i,:)];
  endfor
  
  # run dynamics
  opt = odeset ("RelTol", 1e-04, "AbsTol", 1e-04);
  op = 1;
##  We wish to avoid degenerate inputs - ones that lead to reduction in ode
##  time step and the consequent sluggish ode solver. So we use a stiff solver
##  and we place a try-catch statement. When the stiff solver exceeds the
##  permitted time step, it throws an error that is caught by the try-catch
##  handle. x is replaced with a large value that will result in a high cost
##  and the process continues.

%  try
     #[t,x] = ode15s(@(t,x) dynamics(t,x,p,d_coeff,op),t,x0,opt);
      [t,x] = ode45(@(t,x) dynamics(t,x,p,d_coeff,m,op),t,x0,opt);
%  catch err
      disp('err oops')
##      x = 10*ones(size(d)); #error: 'd' undefined near line 28 column 24
      x = 10*ones(25,30);
%  end_try_catch

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %a global indicator of the optimization process. Meant for debugging.
  global y
  y = y+1
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  # weighting matrices
  Q = 10;     # (8x8)
  Qn = 1000;  # (8x8)
  R = 0.5;    # (4x4)
  
# Calcualte objective function
  # desired state
  yd  = d(1: 4,:);
  dyd = d(5: 8,:);
  # state error (only compare pos and vel) # !!! just compare position
  e = x(:,[10:13,23:26])' - [yd; dyd];
  # control effort
  u = 1/(t(2)-t(1))*gradient(x(:,27:30))';
  costError = 0;
  costControl = 0;
  # get the sum of the errors and control effort over all time instances
  for i = 1:length(t)
    costError   = transpose(e(:,i))*Q*e(:,i) + costError;
    costControl = transpose(u(:,i))*R*u(:,i) + costControl;
  endfor
  # assign an extra penalty for deviation from the desired terminal position
  costTerm = transpose(e(:,end))*Qn*e(:,end);
  # define the objective function
  cost = costError + costControl + costTerm;

endfunction