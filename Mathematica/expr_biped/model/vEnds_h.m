function [x_vEnds_h] = vEnds_h(x)
x_vEnds_h=[x(8),x(9),0.;x(8) + (-0.4905248).*sin(x(4)).*x(11),x(9) +  ...
  0.4905248.*cos(x(4)).*x(11),0.;x(8) + (-0.4905248).*sin(x(4)).*x( ...
  11) + (-0.44363639999999993).*sin(x(5)).*x(12),x(9) + 0.4905248.* ...
  cos(x(4)).*x(11) + 0.44363639999999993.*cos(x(5)).*x(12),0.;x(8) + ...
   (-0.4905248).*sin(x(6)).*x(13),x(9) + 0.4905248.*cos(x(6)).*x(13) ...
  ,0.;x(8) + (-0.4905248).*sin(x(6)).*x(13) + (-0.44363639999999993) ...
  .*sin(x(7)).*x(14),x(9) + 0.4905248.*cos(x(6)).*x(13) +  ...
  0.44363639999999993.*cos(x(7)).*x(14),0.;x(8) + ( ...
  -0.5193791999999999).*sin(x(3)).*x(10),x(9) + 0.5193791999999999.* ...
  cos(x(3)).*x(10),0.];
