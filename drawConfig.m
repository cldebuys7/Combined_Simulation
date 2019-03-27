function [] = drawConfig(t,x)
  # This function draws the configuration at one time instance.
  # input:  t - time vector
  #         x - state vector
  #
  # output: plot of the configuration
  
  # Assign state vector to some easier to read variables (h - human, e - exo)
  [qh, qe, dqh, dqe] = getState(x);
  
  # Get the link end positions of human and exoskeleton, where each row of
  #  qEnds corresponds to x,y,z coordinates of one link end
  ph = qEnds_h(qh); # [hip; stKnee; stFoot; swKnee; swFoot; shoulder]
  pe = qEnds_e(qe); # [hip; stKnee; stFoot; swKnee; swFoot]
  
  # Plot a line from each link end to another link end
  figure(7)
  axis square
##  xlim([-0.8 0])
##  ylim([-0.1 1.3])
  xlim([-0.8 1.3])
  ylim([-0.1 2])
  hold on;
  # plot human
  plot([ph(1,1),ph(6,1)],[ph(1,2),ph(6,2)],'Color','k','LineWidth',2) # torso
  plot([ph(1,1),ph(2,1)],[ph(1,2),ph(2,2)],'Color','k','LineWidth',2) # stThigh
  plot([ph(2,1),ph(3,1)],[ph(2,2),ph(3,2)],'Color','k','LineWidth',2) # stShank
  plot([ph(1,1),ph(4,1)],[ph(1,2),ph(4,2)],'Color','k','LineWidth',2) # swThigh
  plot([ph(4,1),ph(5,1)],[ph(4,2),ph(5,2)],'Color','k','LineWidth',2) # swShank
  # plot exoskeleton
  plot([pe(1,1),pe(6,1)],[pe(1,2),pe(6,2)],'Color','g','LineWidth',2) # torso
  plot([pe(1,1),pe(2,1)],[pe(1,2),pe(2,2)],'Color','g','LineWidth',2) # stThigh
  plot([pe(2,1),pe(3,1)],[pe(2,2),pe(3,2)],'Color','g','LineWidth',2) # stShank
  plot([pe(1,1),pe(4,1)],[pe(1,2),pe(4,2)],'Color','g','LineWidth',2) # swThigh
  plot([pe(4,1),pe(5,1)],[pe(4,2),pe(5,2)],'Color','g','LineWidth',2) # swShank  
  
%  %Creates a walking tile for every 10th increment
%    %set limits of the plot   
%    %q is the state vector
%
%    %the increment of k determines the instances plotted
%    for k = 1:10:size(q,1)
%
%        %calculate position of link ends given state vector q. qEnds and 
%        %qCOM are functions created using mathematica. 
%        %They returns an arrays consisting of the x and y positions of the 
%        %link ends and COM
%        p = qEnds(q(k,:))+[0.1,0,0]*(k-1);
%        COM = qCOM(q(k,:))+[0.1,0,0]*(k-1);
%
%        hold on;
%        %plot all links upto swing ankle            
%        for j = 1:2
%            %Stance leg
%            plot([p(j,1),p(j+1,1)],...
%                [p(j,2),p(j+1,2)],'Color','b','LineWidth',2);
%            %Swing leg
%            plot([p(j+2,1),p(j+3,1)],...
%                [p(j+2,2),p(j+3,2)],'Color','k','LineWidth',2);
%        end
%        %plot torso
%        plot([p(3,1),COM(1,1)],...
%            [p(3,2),COM(1,2)],'Color','k','LineWidth',2)
%        %plot the COM position
%        scatter(COM(:,1),COM(:,2),'MarkerEdgeColor','r','LineWidth',2);
%    end
%    title('Walking tile with every 10th increment') 
  
endfunction