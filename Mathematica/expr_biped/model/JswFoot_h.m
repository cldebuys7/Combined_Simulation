function [x_JswFoot_h] = JswFoot_h(x)
x_JswFoot_h=[1.,0.,0.,0.,0.,(-0.4905248).*sin(x(6)),(-0.44363639999999993).* ...
  sin(x(7));0.,1.,0.,0.,0.,0.4905248.*cos(x(6)), ...
  0.44363639999999993.*cos(x(7));0.,0.,0.,0.,0.,0.,0.];