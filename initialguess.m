function q_guess = initialguess(tspan,SwAnkPos,SwAnkVel,HipPos)    
    %Initial guess state vector x
    %x = { r_x, r_y, q1, q2, q3, q4, 
    % d{r_x}, d{r_y}, d{q1}, d{q2}, d{q3}, d{q4} }
    %Determining initial guess trajectory
    q_guess = zeros(length(tspan),12);   

    %Knee angle of exoskeleton
    LimbAng = 120*pi/180;
    %Values from mathematica
    a1 = 0.22;  b1 = 0.22;  a2 = 0.3-0.05;  b2 = 0.2; 
    StAnkPos = [-0.1371    0.0703];
    for ndx = 1:size(SwAnkPos,1)
        %Refer file Schematic.png for explanation
        p1 = StAnkPos - StAnkPos;
        p2 = HipPos(ndx,:);
        p3 = SwAnkPos(ndx,:);
        %Calculate l1 l2
        l1 = norm(p2-p1,2);  
        l2 = norm(p2-p3,2);
        %Calculate Chi1 and Chi2
        if p1(1) >= p2(1)
            Chi1 = pi - atan2(p1(1)-p2(1),p2(2)-p1(2));
        else
            Chi1 = pi + atan2(p2(1)-p1(1),p2(2)-p1(2));
        end
        if p3(1) <= p2(1)
            Chi2 = pi + atan2(p2(1)-p3(1),p2(2)-p3(2));
        else
            Chi2 = pi - atan2(p3(1)-p2(1),p2(2)-p3(2));
        end
        %Calculate th1 to th4
        th2 = asin( sin(LimbAng)*(a1+b1)/l1 );
        th1 = pi - LimbAng - th2;
        th4 = asin( sin(LimbAng)*(a1+b1)/l2 );
        th3 = pi - LimbAng - th4;
        %Calcualte initial values for q1 to q4
        q_guess(ndx,1) = p2(1);
        q_guess(ndx,2) = p2(2);
        q_guess(ndx,3) = 2*pi-th2-Chi1+pi/2 - 2*pi;
        q_guess(ndx,5) = 2*pi-th4-Chi2+pi/2 - 2*pi;
        q_guess(ndx,4) = sin(th1)*l1/sin(LimbAng) - a2 - b2;
        q_guess(ndx,6) = sin(th3)*l2/sin(LimbAng) - a2 - b2;
    end
    %Evaluate joint velocities
    q_guess(:,7:12) = 1/(tspan(2)-tspan(1))*gradient(q_guess(:,1:6)')';