function [M,C,G,B] = getGlobalSystemMatrices(M_h,C_h,G_h,B_h, M_e,C_e,G_e,B_e)
  # This function creates the global (combined) system matrices from those of
  # the human and the exoskeleton.
  #
  # input:  M_h - inertia matrix              (human)
  #         C_h - coriolis/centrifugal terms  (human)
  #         G_h - gravity vector              (human)
  #         B_h - input map                   (human)
  #         M_e - inertia matrix              (exoskeleton)
  #         C_e - coriolis/centrifugal terms  (exoskeleton)
  #         G_e - gravity vector              (exoskeleton)
  #         B_e - input map                   (exoskeleton)
  #
  # output: M - inertia matrix                (combined)
  #         C - coriolis/centrifugal terms    (combined)
  #         G - gravity vector                (combined)
  #         B - input map                     (combined)
  
  M = blkdiag(M_h,M_e);
  C = blkdiag(C_h,C_e);
  G = [G_h;G_e];
  B = [B_h;B_e];

endfunction
