function myAnimation(X, Y, Psi, pathX, pathY, area, figNum, obstacles, r, theta, p0)
%MYANIMATION 无人艇编队控制仿真动画函数
%   X, Y - 各USV的x和y坐标历史数据
%   Psi - 各USV的航向角历史数据
%   pathX, pathY - 参考路径坐标
%   area - 显示区域 [xmin xmax ymin ymax]
%   figNum - 图形窗口编号
%   obstacles - 障碍物信息 (positions, radii, repulsion_gain, influence_range)
%   r - 编队半径
%   theta - 编队相位角
%   p0 - 当前参考角度

figure(figNum)

shipNum = length(X(1,:)); % 判断船的个数
N = length(X(:,1)); % 判断数据点个数

dt = 0.005;
color = ['r','g','b','c','m'];
linewid = 1;

X1 = [];
Y1 = [];
pathX1 = [];
pathY1 = [];

k1 = 1;
for k=1:100:N
    hold off
    
    % 绘制参考目标位置和编队圆
    target_pos = [pathX(k,1), pathY(k,1)];
    plot(pathY(k,1), pathX(k,1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); hold on
    quiver(pathY(k,1), pathX(k,1), sin(p0(k)), cos(p0(k)), 2, 'k', 'LineWidth', 2); hold on
    
    % 绘制理想编队位置（随目标移动的理想位置）
    for i = 1:length(theta)
        ideal_pos_x = pathX(k,1) + r(i)*cos(theta(i)+p0(k));
        ideal_pos_y = pathY(k,1) + r(i)*sin(theta(i)+p0(k));
        plot(ideal_pos_y, ideal_pos_x, 'ko', 'MarkerSize', 6); hold on
        line([pathY(k,1) ideal_pos_y], [pathX(k,1) ideal_pos_x], 'Color', 'k', 'LineStyle', '--'); hold on
    end
    
    % 绘制虚拟编队圆
    t_circle = linspace(0, 2*pi, 100);
    x_circle = pathX(k,1) + r(1)*cos(t_circle);
    y_circle = pathY(k,1) + r(1)*sin(t_circle);
    plot(y_circle, x_circle, 'k--'); hold on
    
    % 绘制各USV及其历史轨迹
    for i=1:shipNum
        pos = [X(k,i),Y(k,i)]'; psi = Psi(k,i);
        modelplot(pos, psi, color(i), linewid); hold on
        X1(k1,i) = X(k,i);
        Y1(k1,i) = Y(k,i);
        plot(Y1(:,i), X1(:,i), color(i), 'LineWidth', linewid); hold on 
    end
    
    % 绘制目标路径
    pathX1(k1,1) = pathX(k,1);
    pathY1(k1,1) = pathY(k,1);
    plot(pathY1, pathX1, 'k-', 'LineWidth', linewid); hold on 
    
    % 绘制障碍物
    if ~isempty(obstacles) && isstruct(obstacles)
        for i = 1:size(obstacles.positions, 1)
            obstacle_pos = obstacles.positions(i,:);
            obstacle_radius = obstacles.radii(i);
            influence_range = obstacles.influence_range;
            
            % 绘制障碍物圆形
            rectangle('Position',[obstacle_pos(2)-obstacle_radius, obstacle_pos(1)-obstacle_radius, 2*obstacle_radius, 2*obstacle_radius],...
                      'Curvature',[1,1],...
                      'EdgeColor','r',...
                      'FaceColor',[1 0.8 0.8],...
                      'LineWidth',1.5); hold on
            
            % 绘制影响范围
            rectangle('Position',[obstacle_pos(2)-(obstacle_radius+influence_range), obstacle_pos(1)-(obstacle_radius+influence_range), 2*(obstacle_radius+influence_range), 2*(obstacle_radius+influence_range)],...
                      'Curvature',[1,1],...
                      'EdgeColor','r',...
                      'LineStyle','--',...
                      'LineWidth',1); hold on
        end
    end
    
    k1 = k1 + 1;
    grid on;  
    xlabel('y / m');
    ylabel('x / m');
    title(['无人艇编队控制仿真 (t = ' num2str(k*0.05) ' s)']); % 假设ts=0.05
    axis(area);
    
    % 添加图例
    legend('目标点', '目标航向', '理想位置', 'USV1', 'USV2', 'USV3', '目标路径', 'Location', 'NorthWest');
    
    drawnow;
    pause(dt);
end
end