function particle_now = kld_mcl( particle_last, u, z, m )
% 通过KL-distance控制每次迭代的粒子数量
% particle_last：前一时刻的粒子，4*n
% u：里程计的相对位移，2*3，第一列为前一时刻位姿，第二列为当前时刻位姿，基于全局坐标
% z：激光雷达数据，行向量
% particle_now：迭代后当前时刻的粒子，4*m，m与n不相等

% 设置参数
particle_num = 0;  % 当前粒子数量
particle_num_x = 0;  % 粒子数上限，动态变化
particle_num_min = 50;  % 粒子数最小值，按照论文提供的值设置
k = 0;  % 地图中有粒子的栅格的数量
bin = zeros( size(m) );  % 表征地图中某一栅格是否有粒子，0为否，1为是
% delta = 0.05;  % p( K(p_hat,p)<= error ) = 1-delta
error = 0.2;  % KL-distance的边界值
z_quantile = 1.65;  % 标准正态分布上95%分位点，95%=1-delta
res = 100;  % 地图分辨率

% 通过循环生成新的粒子群，一个粒子一个粒子的生成
% matlab没有do……while，只能这么搞
while true
    % 重采样
    thr = rand;
    qtempsum = 0;
    for i = 1 : size(particle_last, 2)
        qtempsum = qtempsum + particle_last(4,i);
        if qtempsum >= thr
            particle_temp = particle_last(:,i);
            break;
        end
    end

    % 更新
    particle_now(1:3, particle_num+1) = sample_motion_model_odo( u, particle_temp(1:3, :) );
    particle_now(4, particle_num+1) = sensor_model(z, particle_now(1:3, particle_num+1), m);

    % 调整粒子数量
    % 将坐标转换成栅格坐标
    temp_map(1) = 10*res - ceil(particle_now(2, particle_num+1).*res);  % 行数和纵坐标是反着来的
    temp_map(2) = ceil(particle_now(1, particle_num+1).*res);
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

    % 检查栅格状态，并计算粒子数动态上限
    if bin(temp_map(1), temp_map(2)) == 0
        k = k+1;
        bin(temp_map(1), temp_map(2)) = 1;
        if k > 1
            particle_num_x = (k-1)./(2*error) .* ( 1 - 2/(9*(k-1)) + sqrt(2/2/(9*(k-1)).* z_quantile) )^3;
        end
    end
    particle_num = particle_num + 1;

    if ~(( particle_num < particle_num_x ) || ( particle_num < particle_num_min ))
        break;
    end
end

particle_now(4,:) = particle_now(4,:)./sum(particle_now(4,:),2);  % 权重归一化，不然重采样容易出现问题

end