% 定位系统综合
% icp：激光里程计，输入激光雷达扫描数据，输出当前时刻位姿（全局坐标）
% rekf：相对位姿融合，u为imu计算出的速度，z为icp计算出的位姿
% kld_mcl：全局定位，u为相对位移（全局坐标），z为雷达数据
% imu_data和laser_data都是一行一组
% 各种pose都是列向量
close all; clear; clc;

%% 初始化
load('test_data_rot_other.mat');
row = size(laser_data, 1);  % 记录有多少组数据
dt = 0.05;  % 每个数据的间隔为50ms
P = diag([1 1 1]);
particle_num = zeros(1,row+1);
particle_num(1) = 10000;  % 起始粒子数量
particle = zeros(4,particle_num(1));  % 前三行为粒子的位姿，第四行为粒子的权重
particle(1:3,:) = pose(:,1) + sqrt(0.1).*randn(3,particle_num(1));  % 按高斯分布在初始位置附近生成粒子
particle(4,:) = 1/particle_num(1) .* ones(1,particle_num(1));  % 所有的粒子权重保持一致
thr = 0.8;  % 用权重排在前80%的粒子估计位姿

tic;
%% 循环计算位姿
for i = 1:row
    % step.1 根据IMU计算速度
    v = [-imu_data(i,1)*dt, -imu_data(i,2)*dt, imu_data(i,6)]';

    % step.2 激光里程计，旧数据为traget，新数据为source
    if i ~= 1  % 第一个时刻只有一个雷达数据，用不了激光里程计
        [R, t, ~] = icp(laser_data(i-1,:), laser_data(i,:) );
        pose_icp = [ R*pose(1:2, i) + t; atan2(R(2), R(1)) + pose(3,i) ];
    else
        pose_icp = pose(:,i) + v*dt;
    end

    % step.3 相对定位，IMU为控制量（u），里程计为观测量（z）
    [pose_rekf, P] = rekf(pose(:,i), P, v, pose_icp);

    % step.4 全局定位
    particle_now = mcl( particle, [ pose(:,i), pose_rekf ], laser_data(i,:), dist );
    max_weight = max( particle_now(4,:) );
    temp = particle_now( :,particle_now(4,:) >= max_weight*thr );  % 找出符合用于计算位姿条件的粒子
    pose(:,i+1) = sum(temp(1:3,:), 2) ./ size(temp,2);
    particle_num(i+1) = size(particle_now,2);

%     clear particle;
    particle = particle_now;  % 更新粒子群，因为新旧数目不一样必须得完全清空之前的
%     clear particle_now;


end
toc;
%% 绘图检查
err = pose_idea - pose;
err_rmse = sqrt( sum(err.^2, 2)./ size(err,2) )

figure(1);
hold on
show(map);
plot(pose_idea(1,:), pose_idea(2,:));
plot(pose(1,:), pose(2,:));
hold off
legend('实际','估计');
title('定位效果分析');

figure(2);
hold on
plot(pose_idea(3,:));
plot(pose(3,:));
hold off
title('航向角估计分析');
legend('实际','估计');

t = 0:0.05:32.7;
figure(3);
plot(t,particle_num,'LineWidth',1.5);
title('粒子数变化');

figure(4);
subplot(3,1,1);
plot(err(1,:));
title('x轴偏差');
subplot(3,1,2);
plot(err(2,:));
title('y轴偏差');
subplot(3,1,3)
plot(err(3,:));
title('航向角偏差');
