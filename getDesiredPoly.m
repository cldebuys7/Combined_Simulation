function [d] = getDesiredPoly(t,d_coeff,tFinal)
  # This function uses polynomial coefficients to evaluate the polynomials
  # for a single time or vector of times.
  #
  # input:  t       - time (scalar or vector)
  #         d_coeff - desired trajectories (polynomial coefficients)
  #         tFinal  - the final time for which the polynomial fit is valid
  #
  # output: d_coeff - coefficients of polynomial fits
  
#------------------------------------------------------------------------------#  
# #This is how the previous function got the coefficients.                     # 
#                                                                              #
#  for i = 1:4                                                                 #
#    polyp(i,:) = polyfit(t,pos(i,:),polyOrder);                               #
#    polyv(i,:) = polyp(i,1:polyOrder).*(polyOrder:-1:1);                      #
#    polya(i,:) = polyp(i,1:polyOrder-1).*(polyOrder-1:-1:1);                  #
#  end                                                                         # 
#  # each row contains the coefficients of a polynomial                        #
#  d_coeff = [polyp;polyv;polya];                                              #
#------------------------------------------------------------------------------# 

  # make sure t does not exceed tFinal (saturates t if t is a vector)
  tau = min(t,tFinal);
    
  polyp = d_coeff(1:4,:);
  polyv = d_coeff(5:8,:);
  polya = d_coeff(9:12,:);
  # if t is a scalar, each row is the value of a polynomial at a given time
  # if t is a vector, each row is a polynomial fit of a desired trajectory
  for i = 1:4
    yd(i,:) = polyval(polyp(i,:),tau);
    dyd(i,:) = polyval(polyv(i,:),tau);
    ddyd(i,:) = polyval(polya(i,:),tau);
  endfor
  
  d = [yd;dyd;ddyd];
endfunction
