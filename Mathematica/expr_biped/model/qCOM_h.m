function [x_qCOM_h] = qCOM_h(x)
x_qCOM_h=[0.32513137919999996.*cos(x(3)) + x(1),0.32513137919999996.*sin(x( ...
  3)) + x(2),0.;0.2123972384.*cos(x(4)) + x(1),0.2123972384.*sin(x( ...
  4)) + x(2),0.;0.4905248.*cos(x(4)) + 0.19209456119999999.*cos(x(5) ...
  ) + x(1),0.4905248.*sin(x(4)) + 0.19209456119999999.*sin(x(5)) +  ...
  x(2),0.;0.2123972384.*cos(x(6)) + x(1),0.2123972384.*sin(x(6)) +  ...
  x(2),0.;0.4905248.*cos(x(6)) + 0.19209456119999999.*cos(x(7)) + x( ...
  1),0.4905248.*sin(x(6)) + 0.19209456119999999.*sin(x(7)) + x(2), ...
  0.];
