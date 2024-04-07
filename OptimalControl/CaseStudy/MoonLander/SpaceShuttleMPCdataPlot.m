close all;
clear all;
clc;

load OCPwoNoise.csv
SpaceTraj = OCPwoNoise;
% load OCPwNoise.csv
% SpaceTraj = OCPwNoise;
% load MPCwNoise.csv
% SpaceTraj = MPCwNoise;

t = 0:0.01:(size(SpaceTraj, 1)*0.01 - 0.01);
Xloc = SpaceTraj(:, 1);
UTraj = SpaceTraj(:, 2);
T = SpaceTraj(:, 3);
TR = SpaceTraj(:, 4);

%% This is the section for plotting the response
figure(1)
plot(t, Xloc, 'LineWidth',2)
xlabel("Time (s)")
ylabel("h (m)")
set(gca, 'FontSize', 20)
grid on;
set(gcf, 'Position', [500, 200, 700, 450])

figure(2)
plot(t, UTraj, 'LineWidth',2)
xlabel("Time (s)")
ylabel("v (m/s)")
set(gca, 'FontSize', 20)
grid on;
set(gcf, 'Position', [500, 200, 700, 450])


figure(3)
hold on
% plot(t, TR, 'LineWidth',2)
plot(t, T, 'LineWidth',2)
% legend("Predicted", "Real")
% legend box off
xlabel("Time (s)")
ylabel("T (m/s^2)")
set(gca, 'FontSize', 20)
grid on;
box on;
set(gcf, 'Position', [500, 200, 700, 450])

%% This is the section for plotting moon lander movie
FG = imread('MoonLander.jpg');
if size(FG,3) == 1
    FG = repmat(FG,[1 1 3]); 
end
FG = flipud(FG);
h = figure(1);
for i = 1:length(t)
    i
    Xlocation = Xloc(i);
    plot(0,0); hold on
    set(gca,'XColor', 'none','YColor','none')

    image(FG, 'xdata', [0 3],'ydata',[Xlocation + 0 Xlocation + 4.5])
    axis equal
    axis([-0.5, 3.5, -1, 14.5])
    set(gca,'XTick',[])
    set(gca,'YTick',[])
    set(gcf, 'Position',[500, 200, 300, 1000])
    % drawnow
    pause(0.01)
    name = "PlotMPCwNoise\MPC" + num2str(i) + ".png"; % Change here for 
    saveas(h, name)
    hold off;

end

% 