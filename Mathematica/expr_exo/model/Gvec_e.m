function [x_Gvec_e] = Gvec_e(x)
x_Gvec_e=[0.,88.29,0. + 2.943.*cos(x(3)),0. + 5.886.*cos(x(4)) + 9.81.*( ...
  0.22.*(0.5.*cos(x(4)) + (-0.8660254037844386).*sin(x(4))) + cos(x( ...
  4)).*(0.45 + x(5))),0. + 9.81.*sin(x(4)),0. + 5.886.*cos(x(6)) +  ...
  9.81.*(0.22.*(0.5.*cos(x(6)) + (-0.8660254037844386).*sin(x(6))) + ...
   cos(x(6)).*(0.45 + x(7))),0. + 9.81.*sin(x(6))];