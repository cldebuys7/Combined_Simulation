function [x_Mmat_e] = Mmat_e(x)
x_Mmat_e=[9.,0.,(-0.3).*sin(x(3)),(-0.1905255888325765).*(cos(x(4)) +  ...
  6.0884210205451454.*sin(x(4)) + 5.2486388108147795.*sin(x(4)).*x( ...
  5)),cos(x(4)),(-0.1905255888325765).*(cos(x(6)) +  ...
  6.0884210205451454.*sin(x(6)) + 5.2486388108147795.*sin(x(6)).*x( ...
  7)),cos(x(6));0.,9.,0.3.*cos(x(3)),1.1600000000000001.*cos(x(4)) + ...
   (-0.1905255888325765).*sin(x(4)) + cos(x(4)).*x(5),sin(x(4)), ...
  1.1600000000000001.*cos(x(6)) + (-0.1905255888325765).*sin(x(6)) + ...
   cos(x(6)).*x(7),sin(x(6));(-0.3).*sin(x(3)),0.3.*cos(x(3)), ...
  0.17333333333333334,0.,0.,0.,0.;(-0.1905255888325765).*(cos(x(4))  ...
  + 6.0884210205451454.*sin(x(4)) + 5.2486388108147795.*sin(x(4)).* ...
  x(5)),1.1600000000000001.*cos(x(4)) + (-0.1905255888325765).*sin( ...
  x(4)) + cos(x(4)).*x(5),0.,0.5255489583333334 + 0.75.*x(4).^2 + ( ...
  1.12 + (-2.7755575615628914e-17).*sin(2..*x(4))).*x(5) + (1. +  ...
  1.1102230246251565e-16.*cos(2..*x(4))).*x(5).^2,( ...
  -0.1905255888325765),0.,0.;cos(x(4)),sin(x(4)),0.,( ...
  -0.1905255888325765),1.,0.,0.;(-0.1905255888325765).*(cos(x(6)) +  ...
  6.0884210205451454.*sin(x(6)) + 5.2486388108147795.*sin(x(6)).*x( ...
  7)),1.1600000000000001.*cos(x(6)) + (-0.1905255888325765).*sin(x( ...
  6)) + cos(x(6)).*x(7),0.,0.,0.,0.5255489583333334 + 0.75.*x(6).^2  ...
  + (1.12 + (-2.7755575615628914e-17).*sin(2..*x(6))).*x(7) + (1. +  ...
  1.1102230246251565e-16.*cos(2..*x(6))).*x(7).^2,( ...
  -0.1905255888325765);cos(x(6)),sin(x(6)),0.,0.,0.,( ...
  -0.1905255888325765),1.];
