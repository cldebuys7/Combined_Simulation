function [x_Cmat_h] = Cmat_h(x)
x_Cmat_h=[0.,0.,(-16.003876852085757).*cos(x(3)).*x(10),( ...
  -3.1979666231039996).*cos(x(4)).*x(11),(-0.6484920291550799).*cos( ...
  x(5)).*x(12),(-3.1979666231039996).*cos(x(6)).*x(13),( ...
  -0.6484920291550799).*cos(x(7)).*x(14);0.,0.,(-16.003876852085757) ...
  .*sin(x(3)).*x(10),(-3.1979666231039996).*sin(x(4)).*x(11),( ...
  -0.6484920291550799).*sin(x(5)).*x(12),(-3.1979666231039996).*sin( ...
  x(6)).*x(13),(-0.6484920291550799).*sin(x(7)).*x(14);0.,0.,0.,0., ...
  0.,0.,0.;0.,0.,0.,0.,0.09422714621371775.*(3.3758999999999997.* ...
  cos(x(5)).*sin(x(4)) + (-3.3758999999999997).*cos(x(4)).*sin(x(5)) ...
  ).*x(12),0.,0.;0.,0.,0.,0.09422714621371775.*(( ...
  -3.3758999999999997).*cos(x(5)).*sin(x(4)) + 3.3758999999999997.* ...
  cos(x(4)).*sin(x(5))).*x(11),0.,0.,0.;0.,0.,0.,0.,0.,0., ...
  0.09422714621371775.*(3.3758999999999997.*cos(x(7)).*sin(x(6)) + ( ...
  -3.3758999999999997).*cos(x(6)).*sin(x(7))).*x(14);0.,0.,0.,0.,0., ...
  0.09422714621371775.*((-3.3758999999999997).*cos(x(7)).*sin(x(6))  ...
  + 3.3758999999999997.*cos(x(6)).*sin(x(7))).*x(13),0.];
