function [x_now] = sample_motion_model_odo( u, x_last )
% 概率运动模型的采样函数
% 根据上一时刻的位姿与当前时刻的控制生成下一时刻可能的位姿
% u：里程计估计的位姿，第一列为上一时刻，第二列为当前时刻
% x_last：上一时刻位姿，列向量
% 误差参数，由运动导致的里程计误差
a_1 = 0.001;  % 旋转-->旋转
a_2 = 0.001;  % 平移-->旋转
a_3 = 0.001;  % 平移-->平移
a_4 = 0.001;  % 旋转-->平移

delta_rot_1 = atan2( u(2,2)-u(2,1), u(1,2)-u(1,1) )-u(3,1);
delta_trans = sqrt( (u(2,2)-u(2,1))^2+(u(1,2)-u(1,1))^2 );
delta_rot_2 = u(3,2) - u(3,1) -delta_rot_1;

delta_rot_1_hat = delta_rot_1 - sqrt(a_1*delta_rot_1^2 + a_2*delta_trans^2).*randn;
delta_trans_hat = delta_trans - sqrt(a_3*delta_trans^2 + a_4*delta_rot_1^2 + a_4*delta_rot_2^2).*randn;
delta_rot_2_hat = delta_rot_2 - sqrt(a_1*delta_rot_2^2 + a_2*delta_trans^2).*randn;

x_now = x_last + [delta_trans_hat * cos(x_last(3)+delta_rot_1_hat);
                  delta_trans_hat * sin(x_last(3)+delta_rot_1_hat);
                  delta_rot_1_hat + delta_rot_2_hat];
end