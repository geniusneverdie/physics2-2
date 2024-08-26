% 定义常数
g = 9.81; % 重力加速度
L = 1; % 摆长
b = 0.5; % 阻尼系数
A = 0.5; % 驱动力幅度
w = sqrt(g/L); % 驱动力频率

% 定义时间范围
tspan = [0 10];

% 定义初始条件 [theta; omega]
init = [pi/4; 0];

% 定义单摆的运动方程
pendulum_nodamping_nodrive = @(t, Y) [Y(2); -g/L*sin(Y(1))];
pendulum_damping_nodrive = @(t, Y) [Y(2); -g/L*sin(Y(1)) - b*Y(2)];
pendulum_damping_drive = @(t, Y) [Y(2); -g/L*sin(Y(1)) - b*Y(2) + A*cos(w*t)];

% 使用ode45求解
[t1, Y1] = ode45(pendulum_nodamping_nodrive, tspan, init);
[t2, Y2] = ode45(pendulum_damping_nodrive, tspan, init);
[t3, Y3] = ode45(pendulum_damping_drive, tspan, init);

% 绘制速度-位置关系曲线
figure
subplot(3,1,1)
plot(Y1(:,1), Y1(:,2))
title('无阻尼无驱动情形')
xlabel('位置')
ylabel('速度')

subplot(3,1,2)
plot(Y2(:,1), Y2(:,2))
title('有阻尼无驱动情形')
xlabel('位置')
ylabel('速度')

subplot(3,1,3)
plot(Y3(:,1), Y3(:,2))
title('有阻尼有驱动情形')
xlabel('位置')
ylabel('速度')