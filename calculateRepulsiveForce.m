function repulsive_force = calculateRepulsiveForce(usv_position, obstacles, USV_positions, usv_idx)
    % 计算针对障碍物和其他USV的斥力
    % usv_position: 当前USV的位置 [x, y]
    % obstacles: 障碍物信息 (positions, radii, repulsion_gain, influence_range)
    % USV_positions: 所有USV的位置
    % usv_idx: 当前USV的索引

    repulsive_force = [0; 0];

    % 控制避障斥力的平滑系数
    smooth_factor = 0.5; % 通过此系数平滑反应

    % 障碍物斥力
    if ~isempty(obstacles) && isstruct(obstacles)
        obs_positions = obstacles.positions;
        obs_radii = obstacles.radii;
        repulsion_gain = obstacles.repulsion_gain;
        influence_range = obstacles.influence_range;

        for i = 1:size(obs_positions, 1)
            obs_pos = obs_positions(i, :)';
            obs_radius = obs_radii(i);

            % 计算USV与障碍物的距离
            distance_vector = usv_position - obs_pos;
            distance = norm(distance_vector);

            % 如果USV在障碍物的影响范围内，计算斥力
            if distance < influence_range + obs_radius
                % 障碍物表面到USV的距离
                dist_to_surface = distance - obs_radius;

                % 避免距离太小导致斥力过大
                if dist_to_surface < 0.5
                    dist_to_surface = 0.5;
                end

                % 计算斥力 (随距离减小而增加)
                force_magnitude = repulsion_gain * (1/dist_to_surface - 1/influence_range) * (1/dist_to_surface^2);

                % 如果距离小于影响范围且大于0，应用斥力
                if dist_to_surface < influence_range && dist_to_surface > 0
                    % 计算单位方向向量 (从障碍物指向USV)
                    if distance > 0
                        direction = distance_vector / distance;

                        % 应用斥力
                        repulsive_force = repulsive_force + force_magnitude * direction;
                    end
                end
            end
        end
    end

    % 计算与其他USV的斥力
    % 其他USV的避免碰撞也需要考虑
    for i = 1:size(USV_positions, 1)
        if i ~= usv_idx
            other_usv_pos = USV_positions(i, :)';
            distance_vector = usv_position - other_usv_pos;
            distance = norm(distance_vector);

            % 设置一个合适的安全距离
            safe_distance = 5;  % 安全距离可调节

            if distance < safe_distance
                force_magnitude = repulsion_gain * (1 / (distance^2));
                if distance > 0
                    direction = distance_vector / distance;
                    repulsive_force = repulsive_force + force_magnitude * direction;
                end
            end
        end
    end

    % 引入平滑因子来减少突然的变化
    repulsive_force = smooth_factor * repulsive_force;
end
