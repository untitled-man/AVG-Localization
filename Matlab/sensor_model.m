function [q] = sensor_model(laser_data, pose_rob, dist)
%  似然域传感器模型，返回p(z_t|x_t)
%  laser_data：雷达数据，极坐标形式，1*n
%  pose_rob：机器人当前时刻的位姿（可能是假设的），1*3
%  m：地图中每个栅格的dist
q = 1;
z_hit = 1;
% z_max = 0;
% z_random = 0;
sigma_hit = 0.2;  % 雷达数据的标准差，matlab的模型里用的是0.2
cols = size(laser_data, 2);
res = 100;  % 地图的分辨率
z = zeros(2, cols);
theta = -pi/2: pi/360: pi/2+pi/360;
z(1,:) = laser_data .* cos(theta);
z(2,:) = laser_data .* sin(theta);  % 雷达坐标系下的数据点（笛卡尔坐标）

for k = 1:cols
    if laser_data(k) ~= 0  % 雷达数据无效时返回的是0
        % 将激光终点的坐标从传感器坐标转换到全局坐标
        temp = [cos(pose_rob(3)) -sin(pose_rob(3));
                    sin(pose_rob(3))  cos(pose_rob(3));]*z(:,k) + pose_rob(1:2,1);
        % 将直角坐标转换成栅格位置
        temp_map(1) = 10*res - ceil(temp(2).*res);  % 行数和纵坐标是反着来的
        temp_map(2) = ceil(temp(1).*res);

        if temp_map(1) < 1
            temp_map(1) = 1;
        elseif temp_map(1) >1000
            temp_map(1) = 1000;
        end

        if temp_map(2) < 1
            temp_map(2) = 1;
        elseif temp_map(2) >1000
            temp_map(2) = 1000;
        end

%         p_hit = normpdf(dist(temp_map(1),temp_map(2)), 0, sigma_hit);
        p_hit = dist(temp_map(1), temp_map(2));
        q = q*z_hit*p_hit;
    end
end