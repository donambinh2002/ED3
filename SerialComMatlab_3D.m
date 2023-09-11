%% Serial COM
clc; clear; close all; format compact; format shortG
s = serialport('COM3', 9600);
configureTerminator(s, "CR/LF");

%% Figure
screen_property = get(0,'screensize');
h = figure('outerposition', ...
    [0, screen_property(4)/2, ...
    screen_property(3)/2, screen_property(4)/2]);
grid on; hold on;

O  = [0,0,0]';
e1 = [0,0,1]';
e2 = [1,0,0]';
e3 = [0,1,0]';
p1 = plot3(  [O(1), e1(1)], ...
    [O(2), e1(2)], ...
    [O(3), e1(3)], ...
    'LineWidth',2,'Color','r');
p2 = plot3(  [O(1), e2(1)], ...
    [O(2), e2(2)], ...
    [O(3), e2(3)], ...
    'LineWidth',2,'Color','g');
p3 = plot3(  [O(1), e3(1)], ...
    [O(2), e3(2)], ...
    [O(3), e3(3)], ...
    'LineWidth',2,'Color','b');

% patch([0, 1, 0],[0, 0, 1],[0, 0, 0],'r', 'alpha')
grid on;
view(-30,30); axis equal
xlabel('y','FontSize',16);
ylabel('z','FontSize',16);
zlabel('x','FontSize',16);
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);
%% Animation
flush(s);
% Initialization
data  = str2double(strsplit(readline(s), ','))
time_start = data(1);
while 1
    data  = str2double(strsplit(readline(s), ','))
    time = data(1);
    if (time-time_start)>=20
        time_start = time;
        %         th1 = data(2);
        %         th2 = data(3);
        %         th3 = data(4);
        th1 = 0;
        th2 = data(2);
        th3 = data(3);
        %         R1 = [  1, 0, 0;
        %             0, cosd(th1), -sind(th1);
        %             0, sind(th1),  cosd(th1)];
        %         R2 = [  cosd(th2), 0, sind(th2);
        %             0, 1, 0;
        %             -sind(th2), 0, cosd(th2)];
        %         R3 = [  cosd(th3), -sind(th3), 0;
        %             sind(th3),  cosd(th3), 0;
        %             0, 0, 1];
        R1 = [  cosd(th1), -sind(th1), 0;
            sind(th1),  cosd(th1), 0;
            0, 0, 1];
        R2 = [  1, 0, 0;
            0, cosd(th2), -sind(th2);
            0, sind(th2),  cosd(th2)];
        R3 = [  cosd(th3), 0, sind(th3);
            0, 1, 0;
            -sind(th3), 0, cosd(th3)];
        e1_pr = R3*R2*R1*e1;
        e2_pr = R3*R2*R1*e2;
        e3_pr = R3*R2*R1*e3;

        set(p1, 'XData', [O(1), e1_pr(1)], ...
            'YData', [O(2), e1_pr(2)], ...
            'ZData', [O(3), e1_pr(3)])
        set(p2, 'XData', [O(1), e2_pr(1)], ...
            'YData', [O(2), e2_pr(2)], ...
            'ZData', [O(3), e2_pr(3)])
        set(p3, 'XData', [O(1), e3_pr(1)], ...
            'YData', [O(2), e3_pr(2)], ...
            'ZData', [O(3), e3_pr(3)])
        drawnow;
    end
end