function modelplot(pos, psi, color, linewid)
% MODELPLOT - 实时绘制船舶几何模型的增强版函数
%
% 输入:  
%   pos: 船舶在惯性坐标系{n}中的实时位置, pos=[x y]' 
%   psi: 实时偏航角
%   color: 船体轮廓的颜色字符串
%   linewid: 轮廓线宽
%
% 输出: 无
%
% 原作者: 曲寅松
% 改进者: Claude
% 最后更新: 2025年3月10日
%

%% 输入验证
if nargin ~= 4 
    error('输入参数必须为4个');
end
if length(pos) ~= 2 
    error('位置向量必须有2个元素');
end
if length(psi) ~= 1 
    error('偏航角必须是标量');
end

%% 船舶几何参数
L = 1.8;     % 长度
W = 1.2;     % 宽度
L_bow = 0.7; % 船首长度（尖锐前部）
L_stern = 0.3; % 船尾形状参数

%% 定义更详细的船舶形状（船体固定坐标系）
% 创建更流线型的船形，带有弧形船首和船尾
num_points = 16;  % 增加点数以获得更平滑的曲线

% 初始化船体点阵列
xb = zeros(2, num_points);

% 船首部分（尖锐前部）
bow_angle = linspace(-pi/2, pi/2, 5);
for i = 1:5
    xb(1,i) = L/2;
    xb(2,i) = W/2 * sin(bow_angle(i));
end

% 右舷（右侧）
xb(:,6) = [L/2-L_bow; W/2];
xb(:,7) = [0; W/2];
xb(:,8) = [-L/2+L_stern; W/2-0.1];

% 船尾（后部）
xb(:,9) = [-L/2; W/4];
xb(:,10) = [-L/2; -W/4];

% 左舷（左侧）
xb(:,11) = [-L/2+L_stern; -W/2+0.1];
xb(:,12) = [0; -W/2];
xb(:,13) = [L/2-L_bow; -W/2];

% 连接回第一个点
xb(:,14:16) = xb(:,1:3);

%% 从船体固定坐标系到惯性坐标系的旋转矩阵
Rb_n = [cos(psi) -sin(psi);
        sin(psi) cos(psi)]; 

%% 将点转换到惯性坐标系
xn = zeros(2, num_points);
for i = 1:num_points
    xn(:,i) = Rb_n * xb(:,i) + pos;
end

%% 添加装饰元素
% 驾驶舱/船桥在船体坐标系中的位置
cabin_length = L/4;
cabin_width = W/2;
cabin_front = L/6;
cabin_back = cabin_front - cabin_length;

cabin_x = [cabin_front, cabin_front, cabin_back, cabin_back];
cabin_y = [cabin_width/2, -cabin_width/2, -cabin_width/2, cabin_width/2];

% 将船桥转换到惯性坐标系
cabin_xn = zeros(2, 4);
for i = 1:4
    cabin_xn(:,i) = Rb_n * [cabin_x(i); cabin_y(i)] + pos;
end

%% 绘制USV
% 提取绘图坐标
N = zeros(1, num_points);
E = zeros(1, num_points);
for i = 1:num_points
    N(i) = xn(1,i);
    E(i) = xn(2,i);
end

% 绘制船体轮廓
plot(E, N, color, 'LineWidth', linewid);

% 用轮廓颜色的较浅色调填充船体
colormap = get(gca, 'ColorOrder');
if strcmp(color(end), '-')
    hull_color = color(1:end-1);
else
    hull_color = color;
end

% 根据轮廓颜色创建较浅的填充颜色
switch hull_color
    case 'r'
        fill_color = [1 0.8 0.8];  % 浅红色
    case 'g'
        fill_color = [0.8 1 0.8];  % 浅绿色
    case 'b'
        fill_color = [0.8 0.8 1];  % 浅蓝色
    otherwise
        fill_color = [0.9 0.9 0.9]; % 浅灰色
end

% 填充船体和船桥
fill(E, N, fill_color);
fill(cabin_xn(2,:), cabin_xn(1,:), [0.7 0.7 0.7]);

% 添加方向指示器（指向前方的小线）
dir_length = L/3;
dir_end = Rb_n * [dir_length; 0] + pos;
plot([pos(2), dir_end(2)], [pos(1), dir_end(1)], [hull_color, ':'], 'LineWidth', linewid/2);

% 保持图形不影响其他绘图
hold on;
end