clc
clear 
close all

ts = 0.05;
tfinal = 600;
Ns = tfinal/ts;

%% parameters initializition
% USV states - reduced to 3 USVs (1 leader, 2 followers)
USV1.x0 = [0 0 0 0 -5 90*pi/180]';  % Leader
USV2.x0 = [0 0 0 14 -5 90*pi/180]';  % Follower 1
USV3.x0 = [0 0 0 -10 -5 90*pi/180]'; % Follower 2

% USV inputs
USV1.tauc=[0 0]';
USV2.tauc=[0 0]';
USV3.tauc=[0 0]';

% 添加障碍物参数
obstacles = struct(...
    'positions', [      % 第一个障碍物位于前进路径上
                  20  40;     % 第二个障碍物位于路径右侧
                  5   60;
                  -15 80;
                 %  ]
                 ],... % 第三个障碍物位于路径左侧
    'radii', [2 2 2],... % 障碍物半径增大，增加避障难度
    'repulsion_gain', 4.0,... % 增大斥力增益，使避障效果更明显
    'influence_range', 15);   % 增大影响范围，使USV提前开始避障

% desired formation - modified for 3 USVs
%--------------------------/
%   USV2     
%     
%             USV1 (Leader)
%     
%   USV3
%------------------------
%----------
% Pijd = [p11d,p12d,p13d;
%         p21d,p22d,p23d;
%         p31d,p32d,p33d];
% Pi0d = diag{[p10d,p20d,p30d]};
p11d = [0;0]; p12d = [0;0]; p13d = [0;0];
p21d = [6;-6]; p22d = [0;0]; p23d = [0;0];
p31d = [-6;-6]; p32d = [0;0]; p33d = [0;0];
Pijd = [p11d,p12d,p13d;
        p21d,p22d,p23d;
        p31d,p32d,p33d];
p10d = [0;0]; p20d = [0;0]; p30d = [0;0];
Pi0d = [p10d;p20d;p30d];

% communication networks based on Laplacian matrix
I21 = [1 1]';
I2 = diag([1 1]);

% Modify adjacency matrix for 3 USVs with leader-follower structure
A = [0 0 0;
     1 0 0;
     1 0 0];
B = diag([1 0 0]);
A_bar = Kerector(A,I2);
B_bar = Kerector(B,I2);
D = diag(sum(A,2));
L = D-A;
H = L+B;
H_bar = Kerector(H,I2);

I31 = [1;1;1]; % Modified for 3 USVs

for k=1:Ns
   % time series
   t = (k-1)*ts;
   
   % path info
   % pc = 1: straight line
   % pc = 2: Wang 2021
   % pc = 3: circle
   % pc = 4: straight line + circle
   % pc = 5: cos
   if t == 0
      pc = 4;
      ud = 0.3; 
      delta0 = 0.1;
      z_w = 0;
   end
   tau_w = [0.3*sin(0.3*t)*cos(0.2*t)+0.2+0.1*randn(),...
            0.1*sin(0.3*t)*cos(0.2*t)+0.1*randn(),...
            0.1*sin(0.2*t)*cos(0.3*t)+0.1*randn()]';
   % USV states update - reduced to 3 USVs
   [USV1.x,USV1.tau,USV1.f] = ASV1(USV1.x0, USV1.tauc, tau_w, ts);
   [USV2.x,USV2.tau,USV2.f] = ASV2(USV2.x0, USV2.tauc, tau_w, ts);
   [USV3.x,USV3.tau,USV3.f] = ASV3(USV3.x0, USV3.tauc, tau_w, ts);

   % USV states
   USV1.eta = [USV1.x(4) USV1.x(5) USV1.x(6)]';
   USV2.eta = [USV2.x(4) USV2.x(5) USV2.x(6)]';
   USV3.eta = [USV3.x(4) USV3.x(5) USV3.x(6)]';
    
   USV1.nu = [USV1.x(1) USV1.x(2) USV1.x(3)]'; USV1.nu_bar = [USV1.x(1) USV1.x(3)]';
   USV2.nu = [USV2.x(1) USV2.x(2) USV2.x(3)]'; USV2.nu_bar = [USV2.x(1) USV2.x(3)]';
   USV3.nu = [USV3.x(1) USV3.x(2) USV3.x(3)]'; USV3.nu_bar = [USV3.x(1) USV3.x(3)]';
   
   % Path Info
   [w,theta,vs,p0,p0_dw] = PathInfo(pc, z_w, ud, ts);
   p0_dw_bar = [p0_dw',p0_dw',p0_dw']'; % Modified for 3 USVs
   
   % P = [p1;p2;p3]; - Modified for 3 USVs
   p1 = [USV1.eta(1);USV1.eta(2)];
   p2 = [USV2.eta(1);USV2.eta(2)];
   p3 = [USV3.eta(1);USV3.eta(2)];
   P = [p1;p2;p3];
   p0_bar = Kerector(I31,p0); % Modified for 3 USVs
   
   % calculate formation errors
   % z1 = [z_usv1, z_usv2, z_usv3];
   z1 = H_bar*(P-p0_bar)-B_bar*Pi0d-sum(Kerector(A,I21).*Pijd,2);
   R_psi1 = [cos(USV1.eta(3)) -sin(USV1.eta(3));
             sin(USV1.eta(3)) cos(USV1.eta(3))];
   R_psi2 = [cos(USV2.eta(3)) -sin(USV2.eta(3));
             sin(USV2.eta(3)) cos(USV2.eta(3))];
   R_psi3 = [cos(USV3.eta(3)) -sin(USV3.eta(3));
             sin(USV3.eta(3)) cos(USV3.eta(3))];
   R_psi = [R_psi1',zeros(2,4);
            zeros(2,2),R_psi2',zeros(2,2);
            zeros(2,4),R_psi3'];
   z2 = R_psi*z1;
   d = Kerector(I31,[delta0,0]'); % Modified for 3 USVs
   z_bar = z2+d;
   z_w = sum(B*Kerector(I31,p0_dw_bar'*R_psi*z_bar),1);
   
   % Guidance
   di_bar = sum(A+B,2);
   h1 = diag([di_bar(1),delta0]);
   h2 = diag([di_bar(2),delta0]);
   h3 = diag([di_bar(3),delta0]);

    % 记录所有USV位置
    USV_positions = [USV1.x(4:5)'; 
                    USV2.x(4:5)';
                    USV3.x(4:5)'];
   
   [USV1.nu_c,USV1.sigmahat] = Guidance1(h1, B(1,1), z_bar(1:2,1), USV1.nu_bar, vs, R_psi1, theta, USV1.nu(3), p0, ts,obstacles, USV_positions, 1);
   [USV2.nu_c,USV2.sigmahat] = Guidance2(h2, B(2,2), z_bar(3:4,1), USV2.nu_bar, vs, R_psi2, theta, USV2.nu(3), p0, ts,obstacles, USV_positions, 2);
   [USV3.nu_c,USV3.sigmahat] = Guidance3(h3, B(3,3), z_bar(5:6,1), USV3.nu_bar, vs, R_psi3, theta, USV3.nu(3), p0, ts,obstacles, USV_positions, 3);
   
   % calculate guidance disturbances
   uj = [USV1.nu(1),USV2.nu(1),USV3.nu(1)]'; % Modified for 3 USVs
   vj = [USV1.nu(2),USV2.nu(2),USV3.nu(2)]'; % Modified for 3 USVs
   if t==0
       Pijdf = Pijd;
       Pi0df = Pi0d;
   end
   Pijdf_dot = -(Pijdf-Pijd)/0.1;
   Pijdf = Pijdf_dot*ts+Pijdf;
   p1jd_dot = Pijdf_dot(1:2,:); p2jd_dot = Pijdf_dot(3:4,:); p3jd_dot = Pijdf_dot(5:6,:);
   
   Pi0df_dot = -(Pi0df-Pi0d)/0.1;
   Pi0df = Pi0df_dot*ts+Pi0df;
   
   USV1.sigma_1 = -R_psi1'*(A(1,1)*R_psi1*[uj(1),vj(1)]'+A(1,2)*R_psi2*[uj(2),vj(2)]'+A(1,3)*R_psi3*[uj(3),vj(3)]')...
                  -R_psi1'*p2jd_dot(1:2,1);
               
   USV2.sigma_1 = -R_psi2'*(A(2,1)*R_psi1*[uj(1),vj(1)]'+A(2,2)*R_psi2*[uj(2),vj(2)]'+A(2,3)*R_psi3*[uj(3),vj(3)]')...
                  -R_psi2'*p2jd_dot(1:2,1);
               
   USV3.sigma_1 = -R_psi3'*(A(3,1)*R_psi1*[uj(1),vj(1)]'+A(3,2)*R_psi2*[uj(2),vj(2)]'+A(3,3)*R_psi3*[uj(3),vj(3)]')...
                  -R_psi3'*p3jd_dot(1:2,1);
               
   USV1.sigma = USV1.sigma_1+[0,di_bar(1)*vj(1)]'-B(1,1)*(vs-theta)*R_psi1'*p0_dw-B(1,1)*Pi0df_dot(1:2,1);
   USV2.sigma = USV2.sigma_1+[0,di_bar(2)*vj(2)]'-B(2,2)*(vs-theta)*R_psi2'*p0_dw-B(2,2)*Pi0df_dot(3:4,1);
   USV3.sigma = USV3.sigma_1+[0,di_bar(3)*vj(3)]'-B(3,3)*(vs-theta)*R_psi3'*p0_dw-B(3,3)*Pi0df_dot(5:6,1);
   
   % Control
   [USV1.tauc,USV1.fhat] = ctr1(USV1.nu_bar, USV1.nu_c, USV1.tau, USV1.tauc, ts);
   [USV2.tauc,USV2.fhat] = ctr2(USV2.nu_bar, USV2.nu_c, USV2.tau, USV2.tauc, ts);
   [USV3.tauc,USV3.fhat] = ctr3(USV3.nu_bar, USV3.nu_c, USV3.tau, USV3.tauc, ts);
   
   z_bar_norm = [norm(z_bar(1:2)),norm(z_bar(3:4)),norm(z_bar(5:6))]'; % Modified for 3 USVs
   
   xout(k,:) = [t,USV1.x', USV2.x', USV3.x', USV1.tau', USV2.tau', USV3.tau',...
                USV1.tauc', USV2.tauc', USV3.tauc', p0',USV1.nu_c',USV2.nu_c',USV3.nu_c',...
                USV1.sigmahat',USV2.sigmahat',USV3.sigmahat',USV1.sigma',...
                USV2.sigma',USV3.sigma',USV1.fhat',USV2.fhat',USV3.fhat',...
                USV1.f',USV2.f',USV3.f',z_bar_norm'];
    
    
end
%% simulation data
t = xout(:,1);
USV1.x = xout(:,2:7);
USV2.x = xout(:,8:13);
USV3.x = xout(:,14:19);
USV1.tau = xout(:,20:21);
USV2.tau = xout(:,22:23);
USV3.tau = xout(:,24:25);
USV1.tauc = xout(:,26:27);
USV2.tauc = xout(:,28:29);
USV3.tauc = xout(:,30:31);
p0 = xout(:,32:33);
USV1.nu_c = xout(:,34:35);
USV2.nu_c = xout(:,36:37);
USV3.nu_c = xout(:,38:39);
USV1.sigmahat = xout(:,40:41);
USV2.sigmahat = xout(:,42:43);
USV3.sigmahat = xout(:,44:45);
USV1.sigma = xout(:,46:47);
USV2.sigma = xout(:,48:49);
USV3.sigma = xout(:,50:51);
USV1.fhat = xout(:,52:53);
USV2.fhat = xout(:,54:55);
USV3.fhat = xout(:,56:57);
USV1.f = xout(:,58:59);
USV2.f = xout(:,60:61);
USV3.f = xout(:,62:63);
z1_bar_norm = xout(:,64);
z2_bar_norm = xout(:,65);
z3_bar_norm = xout(:,66);


%% PLOTS
% formation
figure(1); hold on

fontsize = 10; fontname = 'Times New Roman';
xrange=[-20 50 150]; yrange = [-20 50 150];
linewid = 1;

Xmax = xrange(3);  Xinterval = xrange(2); Xmin = xrange(1);
Ymax = yrange(3);  Yinterval = yrange(2); Ymin = yrange(1);
plot(USV1.x(:,5),USV1.x(:,4),'r--'); 
plot(USV2.x(:,5),USV2.x(:,4),'g--'); 
plot(USV3.x(:,5),USV3.x(:,4),'b--'); 
plot(p0(:,2),p0(:,1),'k-');

% USV1-3
for k=1:2000:Ns
    pos1 = [USV1.x(k,4) USV1.x(k,5)]'; 
    pos2 = [USV2.x(k,4) USV2.x(k,5)]'; 
    pos3 = [USV3.x(k,4) USV3.x(k,5)]'; 
    modelplot(pos1,USV1.x(k,6),'r-',linewid);
    modelplot(pos2,USV2.x(k,6),'g-',linewid);
    modelplot(pos3,USV3.x(k,6),'b-',linewid);
end

set(gca,'xTick',Xmin:Xinterval:Xmax);
set(gca,'yTick',Ymin:Yinterval:Ymax);
axis([Xmin Xmax,Ymin Ymax]);

xlabel('y (m)','FontSize',fontsize,'FontName',fontname);
ylabel('x (m)','FontSize',fontsize,'FontName',fontname);

hold off


figure(2)
subplot(311);plot(t,USV1.tau(:,1),'r-',t,USV1.tau(:,2),'b-'); ylim([-2 2]);
subplot(312);plot(t,USV2.tau(:,1),'r-',t,USV2.tau(:,2),'b-'); ylim([-2 2]);
subplot(313);plot(t,USV3.tau(:,1),'r-',t,USV3.tau(:,2),'b-'); ylim([-2 2]);
title('force real');

figure(3)
subplot(311);plot(t,USV1.tauc(:,1),'r-',t,USV1.tauc(:,2),'b-'); 
subplot(312);plot(t,USV2.tauc(:,1),'r-',t,USV2.tauc(:,2),'b-');
subplot(313);plot(t,USV3.tauc(:,1),'r-',t,USV3.tauc(:,2),'b-');
title('force command');

figure(4)
subplot(311);plot(t,USV1.nu_c(:,1),'r-',t,USV1.nu_c(:,2),'b-'); 
subplot(312);plot(t,USV2.nu_c(:,1),'r-',t,USV2.nu_c(:,2),'b-');
subplot(313);plot(t,USV3.nu_c(:,1),'r-',t,USV3.nu_c(:,2),'b-');
title('speed command');

figure(5)
subplot(311);plot(t,USV1.sigmahat(:,1),'r-',t,USV1.sigmahat(:,2),'b-'); hold on
plot(t,USV1.sigma(:,1),'g--',t,USV1.sigma(:,2),'c--'); hold off

subplot(312);plot(t,USV2.sigmahat(:,1),'r-',t,USV2.sigmahat(:,2),'b-'); hold on
plot(t,USV2.sigma(:,1),'g--',t,USV2.sigma(:,2),'c--'); hold off

subplot(313);plot(t,USV3.sigmahat(:,1),'r-',t,USV3.sigmahat(:,2),'b-'); hold on
plot(t,USV3.sigma(:,1),'g--',t,USV3.sigma(:,2),'c--'); hold off
title('sigma');

figure(6)
subplot(311);plot(t,USV1.fhat(:,1),'r-',t,USV1.fhat(:,2),'b-'); hold on
plot(t,USV1.f(:,1),'g--',t,USV1.f(:,2),'c--'); hold off
subplot(312);plot(t,USV2.fhat(:,1),'r-',t,USV2.fhat(:,2),'b-'); hold on
plot(t,USV2.f(:,1),'g--',t,USV2.f(:,2),'c--'); hold off
subplot(313);plot(t,USV3.fhat(:,1),'r-',t,USV3.fhat(:,2),'b-'); hold on
plot(t,USV3.f(:,1),'g--',t,USV3.f(:,2),'c--'); hold off
title('f');

figure(7)
subplot(311);plot(t,USV1.x(:,1),'r-');
subplot(312);plot(t,USV2.x(:,2),'r-');
subplot(313);plot(t,USV3.x(:,3),'r-');
title('speed real');

figure(8)
subplot(311);plot(t,USV1.x(:,1)-USV1.nu_c(:,1),'r-',t,USV1.x(:,3)-USV1.nu_c(:,2),'b-');
subplot(312);plot(t,USV2.x(:,1)-USV1.nu_c(:,1),'r-',t,USV2.x(:,3)-USV1.nu_c(:,2),'b-'); 
subplot(313);plot(t,USV3.x(:,1)-USV2.nu_c(:,1),'r-',t,USV3.x(:,3)-USV2.nu_c(:,2),'b-'); 
title('speed tracking errors');

figure(9)
plot(t,z1_bar_norm,'r-',t,z2_bar_norm,'g-',t,z3_bar_norm,'b-');
title('formation errors');

% animation
% 在主循环中积累数据
% 准备动画数据
X = [USV1.x(:,4), USV2.x(:,4), USV3.x(:,4)];
Y = [USV1.x(:,5), USV2.x(:,5), USV3.x(:,5)];
Psi = [USV1.x(:,6), USV2.x(:,6), USV3.x(:,6)];
pathX = p0(:,1);
pathY = p0(:,2);
area = [-10 40 -10 120];

% 定义编队参数，用于动画
r = [norm(p21d), norm(p31d)]; % 从预设的编队位置计算半径
theta = [atan2(p21d(2), p21d(1)), atan2(p31d(2), p31d(1))]; % 计算相位角

% 创建每一步的目标朝向向量
heading = zeros(size(p0,1), 1);
for i = 2:size(p0,1)
    dx = p0(i,1) - p0(i-1,1);
    dy = p0(i,2) - p0(i-1,2);
    heading(i) = atan2(dy, dx);
end
heading(1) = heading(2); % 复制第二个点的朝向到第一个点

% 运行动画，减少帧数以提高性能
step = 1; % 每1步取一帧
myAnimation(X(1:step:end,:), Y(1:step:end,:), Psi(1:step:end,:), ...
           pathX(1:step:end), pathY(1:step:end), [Xmin Xmax Ymin Ymax], 10, ...
           obstacles, r, theta, heading(1:step:end));
%pathX = p0(:,1); pathY = p0(:,2);
%area = [Xmin Xmax Ymin Ymax];
%X = [USV1.x(:,4),USV2.x(:,4),USV3.x(:,4)];
%Y = [USV1.x(:,5),USV2.x(:,5),USV3.x(:,5)];
%Psi = [USV1.x(:,6),USV2.x(:,6),USV3.x(:,6)];
%myAnimation(X, Y, Psi, pathX, pathY, area, 10);

