function J = AltCostFcn(X,U,e,data,params)
% based on the "Alternate Cost Function" specified in the mathworks optimization documentation
    Q = 10*eye(12);
    Q(3,3) = 100;
    Q(6,6) = 100;
    Q(8,8) = 100;
    Q(1,8) = 1000;
    Q(4,8) = 1000;
    Q(2,7) = 1000;
    Q(5,7) = 1000;
    Q(2,9) = 1000;
    Q(5,9) = 1000;
    Q(10,10) = 1000;
    Q(11,11) = 1000;
    Q(12,12) = 1000;
    R = 0.01*eye(4);
    R1 = 0.01*eye(4);

    p = data.PredictionHorizon;
    Y = data.CurrentStates;
    RefY = data.References';
    U_tgt = data.MVTarget';
    J = 0;
    for i = 1:p
        % Calculate the output error e_y(k+i)
        e_y = RefY(:,i) - Y;
        % Calculate the input error e_u(k+i)
        e_u = U_tgt(:,i) - U(i,:)';
        % Calculate the control input change delta_u(k+i)
        if i > 1
            delta_u = U(i,:)' - U(i-1,:)';
        else
            delta_u = zeros(4,1); % For the first time step, there's no previous control input
        end
        % Calculate the cost for this time step
        J_i = e_y'*Q*e_y + e_u'*R*e_u + delta_u'*R1*delta_u;
        % Accumulate the cost for all time steps
        J = J + J_i;
    end
end
