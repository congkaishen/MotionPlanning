close all;
clear all;
clc;
Ph = 5; % Crash Scenario
Pv = 4;

% Ph = 1.5; % Succeed Scenario
% Pv = 6;

dt = 0.02;
X0 = [10 0];
X = X0;
T = [0];
U = [0];
for t = 0: dt: 10
    states = X(end, :);
    x1 = states(1);
    x2 = states(2);

    uin = Ph * (-x1) + Pv * ( - x2) ;
    uin = min(max(0, uin), 3);
    dx = SpaceShuttle(states, uin);
    X = [X; states + dx * dt];
    T = [T; t];
    U= [U; uin];
    if abs(states(end, 1)) < 0.05 && abs(states(end, 2)) < 0.3
        disp("Succeed")
        break;
    end
    if abs(states(end, 1)) < 0.05 && abs(states(end, 2)) > 0.05
        disp("Crash")
        break;
    end

end

SaveMatrix = [T X U];
writematrix(SaveMatrix, "PIDFail2.csv");



function dx = SpaceShuttle(x, u)
    x2 = x(2);
    dx1 = x2;
    dx2 = u - 1.625;
    dx = [dx1 dx2];
end