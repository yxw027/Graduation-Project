%-----------This file aims to use control law to get the 
%-----------input of the second-integrator form state function, i.e.,
%-----------dx1 = x2;
%-----------dx2 = u
%% initialization
clear;
clc;
close all;
target = [100 100];
obs = [30 30;
    80 60];
obs_r = 3;
reactR = 12;
buffer_r = 15;

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
    60 5;
    0 0;
    30 50;
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
F = [20, 0;
    0, 20];

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
B = [0; 1];
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
%% Control law with Rotate Force Artificial Potiential Field
    e1(i, :) = x1_1 - target;
    de1(i, :) = x1_2;
    s1 = e1(i, :) + de1(i, :);

    % control input of agent 1
    if dis_to_obc < reactR
        direction = rotate_force(x1_1, x1_2, obs(inx1, :));
        v1 = -K1 * [e1(i, :); de1(i, :)];
        g = 100 * norm(v1);
        v2 = -g * direction * ((1 / (dis_to_obc - obs_r)) - (1 / (reactR - obs_r)));
        u1(i, :) = v1 + v2;
    else
        u1(i, :) = -K1 * [e1(i, :); de1(i, :)];
    end

    % control input of agent 2
    if dis_to_obc2 < reactR
        direction = rotate_force(x2_1, x2_2, obs(inx2, :));
        v1 = -K2 * [e2(i, :); de2(i, :)] + u1(i, :);
        g = 100 * norm(v1);
        v2 = -g * direction * ((1 / (dis_to_obc2 - obs_r)) - (1 / (reactR - obs_r)));
        u2(i, :) = v1 + v2;
    else
        u2(i, :) = -K2 * [e2(i, :); de2(i, :)] + u1(i, :);
    end

    % control input for agent 3
    if dis_to_obc3 < reactR
        direction = rotate_force(x3_1, x3_2, obs(inx3, :));
        v1 = -K2 * [e3(i, :); de3(i, :)] + u1(i, :);
        g = 100 * norm(v1);
        v2 = -g * direction * ((1 / (dis_to_obc3 - obs_r)) - (1 / (reactR - obs_r)));
        u3(i, :) = v1 + v2;
    else
        u3(i, :) = -K2 * [e3(i, :); de3(i, :)] + u1(i, :);
    end

    %% ----------------Another control law for followers(uncomplete)-----------------
%     e1(i, :) = -epc * APF(x1_1, obs, target);
%     de1(i, :) = x1_2;
%     s1 = e1(i, :) + de1(i, :);
%     
%     u1(i, :) = -k1 * s1 - xite * tanh(s1) - x1_2;
%     
%     if dis_to_obc2 > buffer_r
%         flag2 = false;
%     end
% 
%     if dis_to_obc2 < reactR || flag2
% %             u2(i, :) = x2_1 + APF(x2_1, obs, target);
%         u2(i, :) = APF(x2_1, obs, target);
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
%         u3(i, :) = APF(x3_1, obs, target);
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
    u2(i, :) = saturation_input(u2(i, :));
    u3(i, :) = saturation_input(u3(i, :));
    %% update states
    x1_1 = x1_1 + dt * x1_2;
    x1_2 = x1_2 + dt * u1(i, :);

    x2_1 = x2_1 + dt * x2_2;
    x2_2 = x2_2 + dt * u2(i, :);

    x3_1 = x3_1 + dt * x3_2;
    x3_2 = x3_2 + dt * u3(i, :);
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
% figure(4);
% plot(e2(:, 1), de2(:, 1),  'g', 'Linewidth', 1.5);
% hold on;
% plot(e3(:, 1), de3(:, 1),  'b',  'Linewidth', 1.5);
% hold on;
% fimplicit(@(x, y) x + y,  'k',  'Linewidth', 1.5);
% grid on;
% xlabel('e');
% ylabel('de');
% title('Sliding mode');
% legend('Follower1',  'Follower2',  's = 0',  'Location',  'southwest');
figure(5);
plot(1:1:i, vel1(:, 1),  'r',  'Linewidth', 1.5);
hold on;
plot(1:1:i, vel2(:, 1),  'g',  'Linewidth', 1.5);
hold on;
plot(1:1:i, vel3(:, 1),  'b',  'Linewidth', 1.5);
hold on;
grid on;
xlabel('i');
ylabel('speed');
title('Velocity');
legend('Leader',  'Follower1',  'Follower2');
%% functions
% function u = APF(pos, vel, obc, target)
%     reactR = 10;
%     sig = 10; %constant parameter
%     Wa = 0.02; %constant parameter
%     Wb = 500; %constant parameter
%     k1 = 0.2;
% 
%     dis_to_tar = norm(pos - target);
% 
%     v1 = 2 * Wa * (pos - target + sig * rand(1, 2) .* tanh(pos - target) / dis_to_tar);
%     % v2 = zeros(1, length(obc));
% 
%     for i = 1:length(obc)
%         dis_to_obc = norm(pos - obc(i, :));
% 
%         if dis_to_obc < reactR
%             
%                 v2(i, :) = -2 * Wb * reactR^2 / dis_to_obc^4 * (pos - obc(i, :));
%                 % check if agent, target and obstacle lie on a line
%                 ang = dot(pos - target, pos - obc(i, :)) / (dis_to_obc * dis_to_tar);
%                 if abs(ang - 1) < 0.001
%                     v2(i, :) = Wb * rand(1, 2);
%                 end
%         else
%             v2 = 0;
%         end
% 
%     end
% 
%    v2 = sum(v2);
%     u = -k1 * (v1 + v2);
%     u = saturation(u);
% end

function u = APF(pos, obc, target)
    k1 = 0.2;
    obs_r = 10;
    sig = 10; %constant parameter
    Wa = 0.02; %constant parameter
    W2 = 5; %constant parameter 

    dis_to_tar = norm(pos - target);

    v1 = 2 * Wa * (pos - target + sig * rand(1, 2) .* sign(pos - target) / dis_to_tar);

    v2 = zeros(1, length(obc));

    for i = 1:length(obc)
        dis_to_obc = norm(pos - obc(i, :));

        if dis_to_obc < obs_r
            v2(i, :) = -2 * W2 * obs_r^2 / dis_to_obc^4 * (pos - obc(i, :));
            % check if agent, target and obstacle lie on a line
            ang = dot(pos - target, pos - obc(i, :)) / (dis_to_obc * dis_to_tar);
            if abs(ang - 1) < 0.001
                v2(i, :) = W2 * rand(1, 2);
            end
        else
            v2(i, :) = 0;
        end
    end

    v2 = sum(v2);
    u = -k1 * (v1 + v2);
    u = saturation(u);
end

function u = dvdt(pos, vel)
    k1 = 0.2;
    obs_r = 10;
    sig = 10; %constant parameter
    Wa = 0.02; %constant parameter
    W2 = 5; %constant parameter 

    dis_to_tar = norm(pos - target);

    v1 = 2 * Wa * vel;

    v2 = zeros(1, length(obc));

    for i = 1:length(obc)
        dis_to_obc = norm(pos - obc(i, :));

        if dis_to_obc < obs_r
            v2(i, :) = -2 * W2 * obs_r^2 / dis_to_obc^4 * (pos - obc(i, :));
            % check if agent, target and obstacle lie on a line
            ang = dot(pos - target, pos - obc(i, :)) / (dis_to_obc * dis_to_tar);
            if abs(ang - 1) < 0.001
                v2(i, :) = W2 * rand(1, 2);
            end
        else
            v2(i, :) = 0;
        end
    end

    v2 = sum(v2);
    u = -k1 * (v1 + v2);
    u = saturation(u);
end

function f = rotate_force(pos, vel, obc)
    x = pos(1);
    y = pos(2);
    vel_x = vel(1);
    vel_y = vel(2);
    x0 = obc(1);
    y0 = obc(2);

    psi = atan2(vel_y, vel_x);
    chi = atan2(y0 - y, x0 - x);

    if (mod(psi - chi, 2 * pi) <= pi)
        fxkrc = y - y0;
        fykrc = -x - x0;
        fxkr = fxkrc;
        fykr = fykrc;
    else
        fxkrcc = -y - y0;
        fykrcc = x - x0;
        fxkr = fxkrcc;
        fykr = fykrcc;
    end

    f = [fxkr, fykr];
    f = f / norm(f);
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
    maximum = 80;

    if norm(x) > maximum
        u = maximum * x / norm(x);
    else
        u = x;
    end

end
