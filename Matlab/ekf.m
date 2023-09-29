function [x_now, P_now] = ekf(x_lost, P_lost, u, z)
% 通过EKF估计位姿
% 控制量u为里程计数据
% 观测量z为IMU数据
% x_lost为上一时刻的位姿，(x,y,theta)
% p_lost为上一时刻的置信度矩阵，3*3对角阵

% 设置参数
dt = 0.05;
B = [ cos(x_lost(3)) -sin(x_lost(3)) 0;
      sin(x_lost(3))  cos(x_lost(3)) 0;
      0               0              1;].*dt;  % 控制量变换矩阵，状态转移用的
J = [1 0 ( -u(1)*sin(x_lost(3)) + u(2)*cos(x_lost(3)) )*dt; 
     0 1 (  u(1)*cos(x_lost(3)) - u(2)*sin(x_lost(3)) )*dt;
     0 0 1];  % 雅克比矩阵
R = diag( [0.5,0.5,0.5] );  % 里程计噪声
H = eye(3,3);  % 观测矩阵，不需要线性化
K = eye(3,3);  % 卡尔曼增益
Q = diag( [0.2*10^(-4), 0.2*10^(-4), 0.2*10^(-4)]  );  % IMU噪声协方差矩阵，可实测后更改

% step.1 预测
x_now_bar = x_lost + B*u;
P_now_bar = J*P_lost*J' + Q;

% step.2 更新
K = P_now_bar*H'*inv(H*P_now_bar*H'+R);
x_now = x_now_bar + K*(z -x_now_bar);
P_now = ( eye(3,3) - K*H )*P_now_bar;

end