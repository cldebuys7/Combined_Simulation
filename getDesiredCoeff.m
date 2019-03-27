function [d_coeff] = getDesiredCoeff(t,des,ord) 
  # This function gives the coefficients of a polynomial approximation of the
  # desired trajectories. This allows an ODE to obtain values for the desired
  # trajectories when the time step is between actual data points.
  #
  # input:  t   - time vector
  #         des - desired trajectories
  #         ord - order of the polynomial
  #
  # output: d_coeff - coefficients of polynomial fits
  
  # each column of pos is one time instance
  pos = des(:,11:14)';
  polyOrder = ord;
  
  for i = 1:4
    polyp(i,:) = polyfit(t,pos(i,:),ord);
    polyv(i,:) = polyp(i,1:ord).*(ord:-1:1);
    polya(i,:) = polyp(i,1:ord-1).*(ord-1:-1:1);
  endfor
  # include zeros for higher powers which disappear from the derivative
  polyv_tot = [zeros(4,1), polyv];
  polya_tot = [zeros(4,2), polya];
  # each row contains the coefficients of a polynomial
  d_coeff = [polyp; polyv_tot; polya_tot];

#------------------------------------------------------------------------------#
# #This is how a subsequent function will evaluate the polynomials at tau. 
#
#  tau = min(t,tFinal); 
#  polyp = d_coeff(1:4,:);
#  polyv = d_coeff(5:8,:);
#  polya = d_coeff(9:12,:);
#  # each row is a polynomial fit of a desired trajectory
#  for i = 1:4
#    yd(i,:) = polyval(polyp(i,:),tau);
#    dyd(i,:) = polyval(polyv(i,:),tau);
#    ddyd(i,:) = polyval(polya(i,:),tau);
#  endfor
#------------------------------------------------------------------------------#  
endfunction
