%-----------This file aims to use sliding mode control law to get the 
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

% communication matrix
S1 = [1 0 0];
S2 = [1 1 0];
S3 = [1 0 1];
S = [S1; S2; S3];

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

center = zeros(1, 3);
dis_to_tar = norm(x1_1 - target);

%% main code
% sample time
dt = 0.1;
i = 1;
% LQR control parameters
m = 2;
% mass of the vehicle
xite1 = 0.5;
xite2 = 6;
k1 = 1;
k2 = 12;
epc = 1;
c = 2;
while dis_to_tar > 0.1
    dis_to_tar = norm(x1_1 - target);
    [dis_to_obc, inx1] = min([norm(x1_1 - obs(1, :)); norm(x1_1 - obs(2, :))]);
    [dis_to_obc2, inx2] = min([norm(x2_1 - obs(1, :)); norm(x2_1 - obs(2, :))]);
    [dis_to_obc3, inx3] = min([norm(x3_1 - obs(1, :)); norm(x3_1 - obs(2, :))]);

    %% control inputs    
        
    x2d = x1_1 + F(1, :);
    e2(i, :) = x2_1 - x2d;
    de2(i, :) = x2_2 - x1_2;
    s2 = c * e2(i, :) + de2(i, :);

    x3d = x1_1 + F(2, :);
    e3(i, :) = x3_1 - x3d;
    de3(i, :) = x3_2 - x1_2;
    s3 = c * e3(i, :) + de3(i, :);

    e1(i, :) = x1_1 - target;
    de1(i, :) = x1_2;
    s1 = c * e1(i, :) + de1(i, :);

    % control input of agent 1
    % sliding mode control
    sum1 = get_Si(1, S);
    temp1 = zeros(1, 2);
    temp1 = temp1 + S(1, 2) * (e2(i, :) - e1(i, :));
    temp1 = temp1 + S(1, 3) * (e3(i, :) - e1(i, :));
    u1(i, :) = temp1 ./ sum1;
        
        


    % control input of agent2 and agent3
    % sliding mode control variable
    sum2 = get_Si(2, S);
    temp2 = zeros(1, 2);
    temp2 = temp2 + S(2, 1) * (e1(i, :) - e2(i, :));
    temp2 = temp2 + S(2, 3) * (e3(i, :) - e2(i, :));
    u2(i, :) = temp2 / sum2;

    sum3 = get_Si(3, S);
    temp3 = zeros(1, 2);
    temp3 = temp3 + S(3, 1) * (e1(i, :) - e3(i, :));
    temp3 = temp3 + S(3, 2) * (e2(i, :) - e3(i, :));
    u3(i, :) = temp3 / sum3;


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
    
    pos = [x1_1; x2_1; x3_1];
    center(i, :) = [(i - 1) * dt, sum(pos) / 3.0];

    %% dynamic plot
    p4 = plot(x1_1(1), x1_1(2),  'r.');
    hold on;
    p5 = plot(x2_1(1), x2_1(2),  'g.');
    hold on;
    p6 = plot(x3_1(1), x3_1(2),  'b.');
    hold on;
    if mod(i, 50) == 0
        % draw the formation
        k = convhull(pos);
        plot(pos(k ,1), pos(k, 2), 'b');
        hold on;
    end
    pause(0.00001);
    i = i + 1;
end
% csvwrite('F:\Documents\graduationProject\project\UAV\track2\target1.csv', center);
%% plot
pos = [x1_1; x2_1; x3_1];
k = convhull(pos);
plot(pos(k ,1), pos(k, 2), 'b');
hold on;

t = center(:, 1);
grid on;
xlabel('x(m)');
ylabel('y(m)');
title('Trajectories of 3 agents');
legend([p1 p2 p3 p4 p5 p6], {'Target',  'Obstacle1',  'obstacle2' 'Leader',  'Follower1',  'Follower2'},  'Location',  'northwest');
figure(2);
plot(t, e1(:, 1),  'r',  'Linewidth', 2);
hold on;
plot(t, e2(:, 1),  'g',  'Linewidth', 2);
hold on;
plot(t, e3(:, 2),  'b',  'Linewidth', 2);
grid on;
xlabel('Time(sec)');
ylabel('Tracking error(m)');
title('Tracking errors along x-axis');
legend('Leader',  'Follower1',  'Follower2',  'Location',  'northeast');
figure(3);
plot(t, u1(:, 1),  'r',  'Linewidth', 1);
hold on;
plot(t, u1(:, 2),  'r--',  'Linewidth', 1);
hold on;
plot(t, u2(:, 1),  'g',  'Linewidth', 1);
hold on;
plot(t, u2(:, 2),  'g--',  'Linewidth', 1);
hold on;
plot(t, u3(:, 1),  'b',  'Linewidth', 1);
hold on;
plot(t, u3(:, 2),  'b--',  'Linewidth', 1);
hold on;
grid on;
xlabel('Time(sec)');
ylabel('Control input(N)');
title('Control inputs');
legend('Leader x',  'Leader y',  'Follower1 x',  'Follower1 y',  'Follower2 x',  'Follower2 y',  'Location',  'northeast');
figure(4);
plot(e2(:, 1), de2(:, 1),  'g', 'Linewidth', 1.5);
hold on;
plot(e3(:, 1), de3(:, 1),  'b',  'Linewidth', 1.5);
hold on;
fimplicit(@(x, y) c * x + y,  'k',  'Linewidth', 1.5);
grid on;
xlabel('e');
ylabel('de');
title('Sliding mode');
legend('Follower1',  'Follower2',  's = 0');
figure(5);
plot(t, vel1(:, 1),  'r',  'Linewidth', 1.5);
hold on;
plot(t, vel2(:, 1),  'g',  'Linewidth', 1.5);
hold on;
plot(t, vel3(:, 1),  'b',  'Linewidth', 1.5);
hold on;
grid on;
xlabel('Time(sec)');
ylabel('speed(m/s)');
title('Velocity');
legend('Leader',  'Follower1',  'Follower2');


%% functions
function sum = get_Si(i, S)
    sum = 0;
    for j = 1 : length(S(i))
        sum = sum + S(i, j);
    end
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
    maximum = 0.5;

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
function circle(c, r, color)
    x = c(1) - r;
    y = c(2) - r;
    w = 2 * r;
    h = 2 * r;
    rectangle('position',[x, y, w, h],'curvature',[1,1],'edgecolor',color);
end