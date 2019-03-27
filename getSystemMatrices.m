function [M, C, G, B] = getSystemMatrices(q,dq,m,op)
  # This function creates the system matrices.
  # 
  # input:  q  - position vector
  #         dq - velocity vector
  #         u  - control input
  #         op - selects human (1) or exoskeleton (2)
  #
  # output: M - inertia matrix
  #         C - coriolis/centrifugal terms
  #         G - gravity vector
  #         B - input map
  
  n = length(q);
  
  switch op
    case 1
    # human
    M = Mmat_h(q);
    C = Cmat_h([q;dq]);
    G = Gvec_h(q)';
    B = zeros(n,m);

    case 2
    # exoskeleton
    M = Mmat_e(q);
    C = Cmat_e([q;dq]);
    G = Gvec_e(q)';
    B = zeros(n,m);
    B(end+1-m:end, 1:end) = eye(m);
    
    otherwise
      disp('Invalid input to getSystemMatrices')
      
    endswitch

endfunction
