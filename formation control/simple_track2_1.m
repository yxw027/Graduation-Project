%-----------This file aims to use control law to get the 
%-----------input of the second-integrator form state function, i.e.,
%-----------dx1 = x2;
%-----------dx2 = u
%% initialization
clear;
clc;
close all;
target = [10 10];
obs = [3 3;
    8 6];
obs_r = 0.3;
reactR = 1.2;
buffer_r = 1.5;
m = 2;

p1 = plot(target(1), target(2),  'bx');
hold on;
p2 = plot(obs(1, 1), obs(1, 2),  'ro');
hold on;
circle(obs(1, :), obs_r,  'r');
hold on;
circle(obs(1, :), reactR,  'k');
hold on;
p3 = plot(obs(2, 1), obs(2, 2),  'ro');
hold on;
circle(obs(2, :), obs_r,  'r');
hold on;
circle(obs(2, :), reactR,  'k');
hold on;

% Initial Conditions
x = [0 0;
    0 0;
    6 0.5;
    0 0;
    3 5;
    0 0];
x1_1 = x(1, :);
x1_2 = x(2, :);
x2_1 = x(3, :);
x2_2 = x(4, :);
x3_1 = x(5, :);
x3_2 = x(6, :);
pos = [x1_1; x2_1; x3_1];
k = convhull(pos);
plot(pos(k ,1), pos(k, 2), 'b');
hold on;
% Formation shape that agents are going to follow
F = [2, 0;
    0, 2];

e1 = zeros(1, 2);
e2 = zeros(1, 2);
e3 = zeros(1, 2);
de1 = zeros(1, 2);
de2 = zeros(1, 2);
de3 = zeros(1, 2);
u1 = zeros(1, 2);
u2 = zeros(1, 2);
u3 = zeros(1, 2);
vel1 = zeros(1, 2);
vel2 = zeros(1, 2);
vel3 = zeros(1, 2);
v1 = zeros(1, 2);
v2 = zeros(1, 2);

k1 = 2;
k2 = 2;
k3 = 2;
dis_to_tar = norm(x1_1 - target);
flag2 = false;
flag3 = false;

%% main code
dt = 0.1;
epc = 0.8;
xite = 0.8; % impact the speed of agent 1 when it is closed to the target
i = 1;
% LQR control parameters
A = [0 1;
    0 0];
B = [0; 1 / m];
Q2 = 300 * eye(2);
R2 = 1;
Q1 = 10 * eye(2);
R1 = 20;
K1 = lqr(A, B, Q1, R1);
K2 = lqr(A, B, Q2, R2);

while dis_to_tar > 2
    dis_to_tar = norm(x1_1 - target);
    [dis_to_obc, inx1] = min([norm(x1_1 - obs(1, :)); norm(x1_1 - obs(2, :))]);
    [dis_to_obc2, inx2] = min([norm(x2_1 - obs(1, :)); norm(x2_1 - obs(2, :))]);
    [dis_to_obc3, inx3] = min([norm(x3_1 - obs(1, :)); norm(x3_1 - obs(2, :))]);

    %% control inputs    
        
    x2d = x1_1 + F(1, :);
    e2(i, :) = x2_1 - x2d;
    de2(i, :) = x2_2 - x1_2;
    s2 = e2(i, :) + de2(i, :);

    x3d = x1_1 + F(2, :);
    e3(i, :) = x3_1 - x3d;
    de3(i, :) = x3_2 - x1_2;
    s3 = e3(i, :) + de3(i, :);


    %% ----------------Another control law for followers(uncomplete)-----------------
    
    [u1(i, :), v1(i, :), v2(i, :)] = APF(x1_1, obs, target);
    u1(i, :) = u1(i, :) / m;
    
%     if dis_to_obc2 > buffer_r
%         flag2 = false;
%     end
% 
%     if dis_to_obc2 < reactR || flag2
% %             u2(i, :) = x2_1 + APF(x2_1, obs, target);
%         u2(i, :) = APF(x2_1, obs, target) / m;
%         flag2 = true;
%     else
%         flag2 = false;
%         % sliding mode control
%         u2(i, :) = -k2 * s2 - xite * tanh(s2) + u1(i, :) + x1_2 - x2_2;
%         % LQR control
% %             u2(i, :) = -K2 * [e2(i, :); de2(i, :)] + u1(i, :);
%     end
% 
% %     control input of agent 3
%     if dis_to_obc3 > buffer_r
%         flag3 = false;
%     end
% 
%     if dis_to_obc3 < reactR || flag3
% %             u3(i, :) = x3_1 + APF(x3_1, obs, target);
%         u3(i, :) = APF(x3_1, obs, target) / m;
%         flag3 = true;
%     else
%         flag3 = false;
%         % sliding mode control
%         u3(i, :) = -k3 * s3 - xite * tanh(s3) + u1(i, :) + x1_2 - x3_2;
% 
%         % LQR control
% %             u3(i, :) = -K2 * [e3(i, :); de3(i, :)] + u1(i, :);
%     end

    %% -----------------End of another control law---------------------------


    % set input force limit
    u1(i, :) = saturation_input(u1(i, :));
%     u2(i, :) = saturation_input(u2(i, :));
%     u3(i, :) = saturation_input(u3(i, :));
    %% update states
    x1_1 = x1_1 + dt * x1_2;
    x1_2 = x1_2 + dt * u1(i, :);

%     x2_1 = x2_1 + dt * x2_2;
%     x2_2 = x2_2 + dt * u2(i, :);
% 
%     x3_1 = x3_1 + dt * x3_2;
%     x3_2 = x3_2 + dt * u3(i, :);
    x1_2 = saturation(x1_2);
    x2_2 = saturation(x2_2);
    x3_2 = saturation(x3_2);

    vel1(i, :) = x1_2;
    vel2(i, :) = x2_2;
    vel3(i, :) = x3_2;
    i = i + 1;

    %% dynamic plot
    p4 = plot(x1_1(1), x1_1(2),  'r.');
    hold on;
    p5 = plot(x2_1(1), x2_1(2),  'g.');
    hold on;
    p6 = plot(x3_1(1), x3_1(2),  'b.');
    hold on;
    if mod(i, 50) == 0
        % draw the formation
        pos = [x1_1; x2_1; x3_1];
        k = convhull(pos);
        plot(pos(k ,1), pos(k, 2), 'b');
        hold on;
    end
    pause(0.01);
end

%% plot
pos = [x1_1; x2_1; x3_1];
k = convhull(pos);
plot(pos(k ,1), pos(k, 2), 'b');
hold on;
i = i - 1;
grid on;
xlabel('x');
ylabel('y');
title('Trajectories of 3 agents');
legend([p1 p2 p3 p4 p5 p6], {'Target',  'Obstacle1',  'obstacle2' 'Leader',  'Follower1',  'Follower2'},  'Location',  'northwest');
figure(2);
plot(1:1:i, e1(:, 1),  'r',  'Linewidth', 2);
hold on;
plot(1:1:i, e2(:, 1),  'g',  'Linewidth', 2);
hold on;
plot(1:1:i, e3(:, 2),  'b',  'Linewidth', 2);
grid on;
xlabel('Iteration Times');
ylabel('Tracking error');
title('Tracking errors along x-axis');
legend('Leader',  'Follower1',  'Follower2',  'Location',  'northeast');
figure(3);
plot(1:1:i, u1(:, 1),  'r',  'Linewidth', 1);
hold on;
plot(1:1:i, u1(:, 2),  'r--',  'Linewidth', 1);
hold on;
plot(1:1:i, u2(:, 1),  'g',  'Linewidth', 1);
hold on;
plot(1:1:i, u2(:, 2),  'g--',  'Linewidth', 1);
hold on;
plot(1:1:i, u3(:, 1),  'b',  'Linewidth', 1);
hold on;
plot(1:1:i, u3(:, 2),  'b--',  'Linewidth', 1);
hold on;
grid on;
xlabel('Iteration Times');
ylabel('Control input');
title('Control inputs');
legend('Leader x',  'Leader y',  'Follower1 x',  'Follower1 y',  'Follower2 x',  'Follower2 y',  'Location',  'northeast');
figure(4);
plot(1:1:i, v1(:, 1),  'g', 'Linewidth', 1.5);
hold on;
plot(1:1:i, v2(:, 1),  'b',  'Linewidth', 1.5);
hold on;
grid on;
xlabel('e');
ylabel('de');
title('Sliding mode');
legend('Follower1',  'Follower2');
% figure(5);
% plot(1:1:i, vel1(:, 1),  'r',  'Linewidth', 1.5);
% hold on;
% plot(1:1:i, vel2(:, 1),  'g',  'Linewidth', 1.5);
% hold on;
% plot(1:1:i, vel3(:, 1),  'b',  'Linewidth', 1.5);
% hold on;
% grid on;
% xlabel('i');
% ylabel('speed');
% title('Velocity');
% legend('Leader',  'Follower1',  'Follower2');
%% functions
function [u, v1, v2] = APF(pos, obs, target)
    k1 = 0.2;
    obs_r = 0.3;
    reactR = 1.2;
    sig = 10; %constant parameter
    Wa = 0.003; %constant parameter
    W2 = 0.5; %constant parameter 

    dis_to_tar = norm(pos - target);
    dir = (pos - target) / dis_to_tar;
    sig1 = min(sig, sig * sign(pos - target) / dis_to_tar);

    v1 = Wa * (dis_to_tar^2 * dir + sig1);

    v2 = zeros(1, length(obs));

    for i = 1:length(obs)
        dis_to_obc = norm(pos - obs(i, :));

        if dis_to_obc < reactR
            v2 = -W2 * (reactR^2 / dis_to_obc^2 - 1) * dir;
            % check if agent, target and obstacle lie on a line
            ang = dot(pos - target, pos - obs(i, :)) / (dis_to_obc * dis_to_tar);
            if abs(ang - 1) < 0.001
                v2 = W2 * 10*rand(1, 2);
            end
            break;
        else
            v2 = [0 0];
        end
    end
    u = -k1 * (v1 + v2);
    u = saturation(u);
end

function u = saturation(x)
    maximum = 5;

    if norm(x) > maximum
        u = maximum * x / norm(x);
    else
        u = x;
    end

end

function u = saturation_input(x)
    maximum = 8;

    if norm(x) > maximum
        u = maximum * x / norm(x);
    else
        u = x;
    end

end
