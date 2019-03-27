function u = getControlInput(t, xe, p, d_coeff, f, g)
  # This function is called by dynamics.m to calculate the control input for
  # the exoskeleton given f(x), g(x), and some desired trajectories. The
  # system is assumed to have the form xdot = f(x) + g(x)*u.
  #
  # input:  t - time
  #         x - state vector
  #         p - struct of physical parameters
  #   d_coeff - desired trajectories (polynomial coefficients)
  #         f - system dynamics which are independent from the input
  #         g - system dynamics which are dependent on input
  #
  # output: u - control input

# Feedback Linearization
  # actual state
  [y,dy] = getOutput(xe); #!! may need to be relative angles
  
  # calc desired state from coefficients using current time t
  tFinal = p.stepTime;
  [d] = getDesiredPoly(t,d_coeff,tFinal);
  # desired state
  yd   = d(1: 4); #!! may need to be relative angles
  dyd  = d(5: 8);
  ddyd = d(9:12);
  
  # Lie derivatives of y = h(x) for output of relative degree 2 (Lf2h, LgLfh)
  m = length(y);
  n = length(xe);
  dh_dx = [zeros(m,n-m), eye(m), zeros(4,n)]; # dh(x)/dx, where h(x) = y
  dxdot_dx = [zeros(n), eye(n); zeros(n), zeros(n)];
  dLfh_dx = dh_dx*dxdot_dx; # d(Lfh)/dx, where d^2h/dx^2 = 0 in this case
  Lf2h = dLfh_dx*f;
  LgLfh = dLfh_dx*g;
  cl = cond(LgLfh)
  # controller design
  wn = 1e3;
  Kp = wn^2;
  Kd = 2*wn;  # this choice of Kp, Kd yields critically damped error dynamics
  v = ddyd - Kd*(dy - dyd) - Kp*(y - yd); # psuedo-control #!! ddyd may need to be relative
  u = LgLfh \ (-Lf2h + v);
endfunction
