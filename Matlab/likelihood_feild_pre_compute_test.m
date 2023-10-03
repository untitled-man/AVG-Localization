%% 测试预计算似然域的算法
clear;
clc;

%% 生成测试用的栅格地图
p = zeros(1000,1000);

% 边框
p(1:7,:) = ones(7,1000);
p(994:1000,:) = ones(7,1000);
p(:,1:7) = ones(1000,7);
p(:,994:1000) = ones(1000,7);

% 终点障碍物
p(101:300, 901:915) = ones(200,15);
p(86:100, 701:900) = ones(15,200);

% 随机表示物
p(486:515,331:370) = ones(30,40);

map_temp = binaryOccupancyMap(10,10,100);
x = [8 6.3]';
y = [3 7.6]';
setOccupancy(map_temp, [x y], ones(2,1));
inflate(map_temp, 0.15);
occMatrix = getOccupancy(map_temp);
p = p + occMatrix;

map = binaryOccupancyMap(p,100);  % 10m*10m，一个栅格为0.01m*0.01m

show(map);
%% 迭代计算距离
dist = zeros(1000,1000);  % 存距离信息

for i = 1:1000
    for j = 1:1000
        % 如果是障碍，距离记为0，继续下一个单元
        if p(i,j) == 1
            dist(i,j) = 0;
            continue;
        end

        % 进行到这说明不是障碍，按层寻找最近障碍
        % 距离以栅格中点距离计算
        u = 1;
        while true
            for k = -u:u
                if (i-u)>0 && (j+k)>0 && (i-u)<1001 && (j+k)<1001
                    if p(i-u,j+k) == 1
                        dist(i,j) = sqrt( (u*0.01)^2 + (abs(k)*0.01)^2 );
                    end
                end
            end
            for k = -u:u
                if (i+k)>0 && (j+u)>0 && (i+k)<1001 && (j+u)<1001
                    if p(i+k,j+u) == 1
                        dist(i,j) = sqrt( (u*0.01)^2 + (abs(k)*0.01)^2 );
                    end
                end
            end
            for k = -u:u
                if (i+u)>0 && (j+k)>0 && (i+u)<1001 && (j+k)<1001
                    if p(i+u,j+k) == 1
                        dist(i,j) = sqrt( (u*0.01)^2 + (abs(k)*0.01)^2 );
                    end
                end
            end
            for k = -u:u
                if (i+k)>0 && (j-u)>0 && (i+k)<1001 && (j-u)<1001
                    if p(i+k,j-u) == 1
                        dist(i,j) = sqrt( (u*0.01)^2 + (abs(k)*0.01)^2 );
                    end
                end
            end
            u = u+1;
            if u > 100
                dist(i,j) = 999;
                break;
            end

            if dist(i,j) > 0
                break;
            end
        end
    end
end
