function [sys, x0, str, ts] = plant1(t, x, u, flag, z0)
% input control of agent 1
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
        sizes.NumOutputs = 6;
        sizes.NumInputs = 2;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);
        x0 = z0; % 长度必须和NumContStates一样.初始状态z0需要双击S-Function设置
        str = [];
        ts = [0 0];

        function sys = mdlDerivatives(t, x, u)
            k = 1;
            target = u;
            % track error along x-axis and y-axis
            x1 = [x(1) x(2)];
            x2 = [x(3) x(4)];
            e = x1 - target;
            de = x2;
            % sliding mode
            s = e + de;
            % input of agent 1
            epc = 0.05;
            u1 = -x2 - k*s-tanh(s / epc);
            % state function
            % sys(1): dot_x1
            sys(1) = x2(1);
            sys(2) = x2(2);
            sys(3) = u1(1);
            sys(4) = u1(2);
%             [sys(1), sys(2)] = x2;
%             [sys(3), sys(4)] = u1;
%             sys(1) = x(2);
%             sys(2) = u1;
            
        function sys = mdlOutputs(t, x, u)   
            target = u;
            % track error
            e = x(1) - target;
            de = x(2);
            % sliding mode
            s = e + de;
            % input of agent 1
            epc = 0.05;
            u1 = -x(2) - s -tanh(s / epc);
            % output u1 x1 and x2    
            sys(1) = x(1);
            sys(2) = x(2);
            sys(3) = x(3);
            sys(4) = x(4);
            sys(5) = u1(1);
            sys(6) = u1(2);
