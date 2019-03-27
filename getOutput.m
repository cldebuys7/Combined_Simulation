function [y, dy] = getOutput (x)
  # This function defines the output and its derivative from the state vector
  # of the exoskeleton.
  #
  # input:  x - state vector
  #
  # output: y  - system output
  #         dy - time derivative of the output
  
  y = x(4:7);
  dy = x(11:14);
  
endfunction
