function [R, t, err] = icp(laser_data_old, laser_data)
% 点云ICP配准，将laser_data配准至laser_data_old
% laser_data_old：k-1时刻的扫描数据，traget
% laser_data；k时刻的扫描数据，source
% R,t；配准结果，满足traget = R*source + t

coord = zeros(2, 362);
coord_old  = zeros(2, 362);
T = eye(4,4);
itr = 5;  % 迭代次数上限
thr = 10^(-5);  % 精度要求
err = 100;  % 误差
k = 1;

% 转换为直角坐标
theta = -pi/2:pi/360:pi/2+pi/360;
[ coord(1,:),coord(2,:) ]=pol2cart(theta,laser_data);
[ coord_old(1,:),coord_old(2,:) ]=pol2cart(theta,laser_data_old);

% 刨去无效点
coord(:,laser_data==0)=[];
coord_old(:,laser_data_old==0)=[];

% 调整有效点集数量，保持一致
cols_trix = size( coord,2 );
cols_old_trix = size( coord_old,2 );
trix = cols_trix - cols_old_trix;
if trix > 0  % 抛弃多余的点
    coord_traget = coord_old(:,1:cols_trix-trix);
    coord_source = coord;
elseif trix < 0
    coord_traget = coord_old;
    coord_source = coord(:,1:cols_old_trix+trix);
else
    coord_traget = coord_old;
    coord_source = coord;
end

% 迭代运算R与t
% traget不会变，不用重复归一化
center_traget = sum(coord_traget,2)/size(coord_traget,2);
coord_traget_a = coord_traget - center_traget;
while ( k<itr && err>thr )
    if k ~= 1
        coord_source = coord_source_trans;
    end

    % 归一化
    center_source = sum(coord_source,2)/size(coord_source,2);
    coord_source_a = coord_source - center_source;

    % 以traget为基础，求每个s点距离最近的t点
    kd_tree = KDTreeSearcher(coord_traget_a','BucketSize',10);
    [index, ~] = knnsearch(kd_tree, coord_source_a');

    % 求解R，t
    H = zeros(2,2);
    for i = 1:size(coord_source_a,2)
        H = coord_source_a(:,i)*coord_traget_a(:,index(i))' + H;
    end
    [U,~,V] = svd(H);
    R_k = V*U';
    t_k = center_traget - R_k*center_source;

    % 配准后的计算误差
    coord_source_trans = R_k*coord_source + t_k;
    kd_tree = KDTreeSearcher(coord_traget','BucketSize',10);
    [~, dist] = knnsearch(kd_tree, coord_source_trans');
    err = sum(dist.^2)/size(dist,1);

    % 通过变换矩阵实现多次变换叠加
    k = k+1;
    T_k = [R_k zeros(2,1) t_k;
        0 0 1 0;
        0 0 0 1;];
    T = T_k * T;  
end
R = T(1:2, 1:2);
t = T(1:2, 4);
end