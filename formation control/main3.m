%-----------This file aims to use control law to get the 
%-----------input of the single-integrator form state function, i.e.,
%-----------dx = u;

% initialization
clear;
clc;
close all;
target = [100 100];
lem = 2;
obs = [50 50;
    80 60];
obs_r = 10;
buffer_r = 12;
sig = 50; %constant parameter
Wa = 20; %constant parameter
W2 = 20; %constant parameter
p1 = plot(0.1*target(1), 0.1*target(2),  'bx');
hold on;
p2 = plot(0.1*obs(1, 1), 0.1*obs(1, 2),  'ro');
hold on;
circle(0.1*obs(1, :), 0.1*obs_r,  'r');
hold on;
p3 = plot(0.1*obs(2, 1), 0.1*obs(2, 2),  'ro');
hold on;
circle(0.1*obs(2, :), 0.1*obs_r,  'r');
hold on;

% Initial Conditions
x = [0 0;
    6 0.5;
    3 5];
pos = [x(1, :); x(2, :); x(3, :)];
k = convhull(pos);
plot(pos(k ,1), pos(k, 2), 'b');
hold on;
% Formation shape that agents are going to follow
D = [20, 0;
    0, 20];
k2 = 0.4;
N = 300;
e = zeros(1, 2);
y1 = zeros(1, 2);
vel1 = zeros(1, 2);
vel2 = zeros(1, 2);
vel3 = zeros(1, 2);
i = 1;
dis_to_tar = norm(x(1, :) - target);
flag2 = false;
flag3 = false;
while dis_to_tar > 1.5
% while i < 300
    x = 10 * x;
    [dis_to_obc, ~] = min([norm(x(1, :) - obs(1, :)); norm(x(1, :) - obs(2, :))]);
    [dis_to_obc_2, ~] = min([norm(x(2, :) - obs(1, :)); norm(x(2, :) - obs(2, :))]);
    [dis_to_obc_3, ~] = min([norm(x(2, :) - obs(1, :)); norm(x(2, :) - obs(2, :))]);

    u1 = APF(x(1, :), obs, target);
    % control input of agent 2
    if dis_to_obc_2 > buffer_r
        flag2 = false;
    end

    if dis_to_obc_2 < obs_r || flag2
        u2 = APF(x(2, :), obs, target);
        flag2 = true;
    else 
        flag2 = false;
        e2 = x(2, :) - x(1, :) - D(1, :);
        u2 = -k2 * e2 + u1;
        u2 = saturation(u2);
    end
    % control input of agent 3
    if dis_to_obc_3 > buffer_r
        flag3 = false;
    end

    if dis_to_obc_3 < obs_r || flag3
        u3 = APF(x(3, :), obs, target);
        flag3 = true;
    else 
        flag3 = false;
        e3 = x(3, :) - x(1, :) - D(2, :);
        u3 = -k2 * e3 + u1;
        u3 = saturation(u3);
    end

    u = [u1; u2; u3];
    x = x + u;
    e(i, 1) = norm(x(2, :) - x(1, :) - D(1, :));
    e(i, 2) = norm(x(3, :) - x(1, :) - D(2, :));
    y1(i, :) = x(1, :);
    vel1(i, :) = u1;
    vel2(i, :) = u2;
    vel3(i, :) = u3;
    i = i + 1;
    dis_to_tar = norm(x(1, :) - target);
    x = 0.1 * x;
    p4 = plot(x(1, 1), x(1, 2),  'r.');
    hold on;
    p5 = plot(x(2, 1), x(2, 2),  'g.');
    hold on;
    p6 = plot(x(3, 1), x(3, 2),  'b.');
    hold on;
    if mod(i, 50) == 0
        % draw the formation
        pos = [x(1, :); x(2, :); x(3, :)];
        k = convhull(pos);
        plot(pos(k ,1), pos(k, 2), 'b');
        hold on;
    end
    xlim([0 14]);
    ylim([0 14]);
    pause(0.001);

end
pos = [x(1, :); x(2, :); x(3, :)];
k = convhull(pos);
plot(pos(k ,1), pos(k, 2), 'b');
hold on;
grid on;
xlabel('x(m)');
ylabel('y(m)');
title('Trajectories of 3 agents');
legend([p1 p2 p3 p4 p5 p6], {'Target', 'Obstacle1', 'obstacle2' 'Leader', 'Follower1', 'Follower2'}, 'Location', 'northwest');
figure(2);
e = 0.1 * e;
plot(1:1:i-1, e(:, 1), 'r');
hold on;
plot(1:1:i-1, e(:, 2), 'g');
grid on;
xlabel('Time(sec)');
ylabel('Tracking error(m)');
title('Tracking errors along x-axis');
legend('Follower1', 'Follower2', 'Location', 'northeast');
figure(3);
plot(1:1:i-1, vel1(:, 1), 'r');
hold on;
plot(1:1:i-1, vel1(:, 2), 'r--');
hold on;
plot(1:1:i-1, vel2(:, 1), 'g');
hold on;
plot(1:1:i-1, vel2(:, 2), 'g--');
hold on;
plot(1:1:i-1, vel3(:, 1), 'b');
hold on;
plot(1:1:i-1, vel3(:, 2), 'b--');
hold on;
grid on;
xlabel('Time(sec)');
ylabel('Control input');
title('Control inputs(m/s)');
legend('Leader x', 'Leader y', 'Follower1 x', 'Follower1 y','Follower2 x', 'Follower2 y','Location', 'northeast');

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

function u = saturation(x)
    maximum = 5;
    if norm(x) > maximum
        u = maximum * x / norm(x);
    else
        u = x;
    end
end