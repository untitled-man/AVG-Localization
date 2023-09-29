function [x_now, P_now] = rekf(x_lost, P_lost, u, z)
% 通过EKF估计位姿
% 能够自动调节R，提高自适应能力
% P为方差矩阵，貌似是对角阵，表示对每个估计的置信程度
% 控制量u为IMU数据
% 观测量z为里程计数据
% x_lost为上一时刻的位姿，(x,y,theta)
% p_lost为上一时刻的置信度矩阵，3*3对角阵

% 设置参数
dt = 0.05;
c_0 = 1;
c_1 = 3;
r = [0 0 0]';  % 抗差因子
B = [ cos(x_lost(3)) -sin(x_lost(3)) 0;
    sin(x_lost(3))  cos(x_lost(3)) 0;
    0               0              1;].*dt;  % 控制量变换矩阵，状态转移用的
J = [1 0 ( -u(1)*sin(x_lost(3)) + u(2)*cos(x_lost(3)) )*dt;
    0 1 (  u(1)*cos(x_lost(3)) - u(2)*sin(x_lost(3)) )*dt;
    0 0 1];  % 雅克比矩阵

H = eye(3,3);  % 观测矩阵，不需要线性化
Q = diag( [0.2*10^(-4), 0.2*10^(-4), 0.2*10^(-4)]  );  % IMU噪声协方差矩阵，可实测后更改
R = diag( [0.5, 0.5, 0.5] );  % 里程计噪声

% 1.预测
x_now_bar = x_lost + B*u;
P_now_bar = J*P_lost*J' + Q;

% 2.计算抗差因子
v = z - H*x_now_bar;  % 计算残差
% temp = sqrt( sum( v.^2 )/2 );
% v_bar = abs(v./temp);  % 标准化残差，之后要用绝对值这里就先算完
v_bar = abs( v/std(v) );  % 另一种标准化的办法，效果似乎差不多
for i = 1:3
    if ( (v_bar(i) <= c_0) ||  v(i) == 0 )  % 当v=0时，v_bar就是NaN，导致出错
        r(i) = 1;
    elseif v_bar(i) > c_1
        r(i) = 10*(-30);
    else
        r(i) = c_0/v_bar(i)*( (c_1-v_bar(i))/(c_1-c_0) )^2;
    end
end
R = R * diag(1./r);

% 3.更新
K = P_now_bar*H'/(H*P_now_bar*H'+R);  % 卡尔曼增益
x_now = x_now_bar + K*(z - x_now_bar);
P_now = ( eye(3,3) - K*H )*P_now_bar;

end