function [sys, x0, str, ts] = plant2(t, x, u, flag, z0)
% input control of agent 2
    switch flag
        case 0
            [sys, x0, str, ts] = mdlInitializeSizes(z0);
        case 1
            sys = mdlDerivatives(t, x, u);
        case 3
            sys = mdlOutputs(t, x, u);
        case {2, 4, 9}
            sys = [];
        otherwise
            error(['Unhandled flag = ', num2str(flag)]);
    end

    function [sys, x0, str, ts] = mdlInitializeSizes(z0)
        sizes = simsizes;
        sizes.NumContStates = 4;
        sizes.NumDiscStates = 0;
        sizes.NumOutputs = 4;
        sizes.NumInputs = 6; % need to know postion and velocity of agent1 and control input u1
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        x0 = z0; 
        str = [];
        ts = [0 0];

        function sys = mdlDerivatives(t, x, u)
            k = 1;
            relative_pos = [20 0];
            % position of agent 1
            x1_1 = [u(1) u(2)];
            x1_2 = [u(3) u(4)];
            % control input u1
            u1 = [u(5) u(6)];

            target = x1_1 + relative_pos;
            % tracking error along x-axis and y-axis: position
            x2_1 = [x(1) x(2)];
            % tracking error along x-axis and y-axis: velocity
            x2_2 = [x(3) x(4)];
            e = x2_1 - target;
            de = x2_2 - x1_2;
            % sliding mode
            s = e + de;
            % input of agent 1
            epc = 0.05;
            u2 = -k*s-tanh(s / epc)+u1+x1_2-x2_2;
            % state function
            % sys(1): dot_x2
            sys(1) = x2_2(1);
            sys(2) = x2_2(2);
            sys(3) = u2(1);
            sys(4) = u2(2);
%             [sys(1), sys(2)] = x2;
%             [sys(3), sys(4)] = u1;
%             sys(1) = x(2);
%             sys(2) = u1;
            
        function sys = mdlOutputs(t, x, u)   
            % output u2 x2_1 and x2_2    
            sys(1) = x(1);
            sys(2) = x(2);
            sys(3) = x(3);
            sys(4) = x(4);
 
