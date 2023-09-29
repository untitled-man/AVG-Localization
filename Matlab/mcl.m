function [par_now] = mcl( par_last, u, z, m )
% 蒙特卡罗定位
% par_last：上一时刻的粒子，第1~3行为位姿，第4行为权重
% u：全局坐标下里程计估计出的前一时刻和当前时刻的位姿，3*2
% z：激光激光雷达数据，1*n
% m：预计算完的地图，矩阵
% par_now：经过迭代的粒子
% 当前估算的位置粒子的中心表示
par_now = zeros( size(par_last) );
par_bar = par_now;
par_num = size(par_last,2);  % 粒子数量

for i = 1:par_num
    par_bar(1:3,i) = sample_motion_model_odo( u, par_last(1:3,i) );
    par_bar(4,i) = sensor_model(z, par_bar(1:3,i), m);
end

% 概率归一化，便于后续淘汰
par_bar(4,:) = par_bar(4,:)./sum(par_bar(4,:));

% 重采样，淘汰概率低的粒子
N=length(par_bar(4,:));
for i = 1 : N
    u = rand;
    qtempsum = 0;
    for j = 1 : N
        qtempsum = qtempsum + par_bar(4,j);
        if qtempsum >= u
            par_now(:,i) = par_bar(:,j);
            break;
        end
    end
end

end

