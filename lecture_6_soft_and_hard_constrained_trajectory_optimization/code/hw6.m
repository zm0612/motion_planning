clc
clear
close all
v_max = 150;
a_max = 200;
j_max = 400;
color = ['r', 'b', 'm', 'g', 'k', 'c'];

% %% Use the matlab robotics toolbox to generate B-spine path
% cpts = [50 100 180 250 280; 50 120 150 80 0];
% tpts = [0 5];
% tvec = 0:0.01:5;
% [q, qd, qdd, pp] = bsplinepolytraj(cpts,tpts,tvec);

%% Use standard constrained qp method
% specify the center points of the flight corridor and the region of corridor
path = [50, 50;
    100, 120;
    180, 150;
    250, 80;
    280, 0];

x_length = 100;
y_length = 100;

n_order = 7;   % 8 control points
n_seg = size(path, 1);

corridor = zeros(4, n_seg);
for i = 1:n_seg
    corridor(:, i) = [path(i, 1), path(i, 2), x_length/2, y_length/2]';
end

%% specify ts for each segment
ts = zeros(n_seg, 1);
for i = 1:n_seg
    ts(i,1) = 1;
end

poly_coef_x = MinimumSnapCorridorBezierSolver(1, path(:, 1), corridor, ts, n_seg, n_order, v_max, a_max, j_max);
poly_coef_y = MinimumSnapCorridorBezierSolver(2, path(:, 2), corridor, ts, n_seg, n_order, v_max, a_max, j_max);

%% display the trajectory and cooridor
f1 = plot(path(:,1), path(:,2), '*r','DisplayName','waypoints');
hold on;
for i = 1:n_seg
    plot_rect([corridor(1,i);corridor(2,i)], corridor(3, i), corridor(4,i));
    hold on
end

x_pos = [];
y_pos = [];
idx = 1;

%% #####################################################
% STEP 4: draw bezier curve
for k = 1:n_seg
    for t = linspace(0,1)
        x_pos(idx) = 0.0;
        y_pos(idx) = 0.0;
        for i = 0:n_order
            basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);
            x_pos(idx) = x_pos(idx) + poly_coef_x((k-1)*(n_order+1)+i+1) * basis_p;
            y_pos(idx) = y_pos(idx) + poly_coef_y((k-1)*(n_order+1)+i+1) * basis_p;
        end
        idx = idx + 1;
    end
end

for k=1:n_seg
    f1 = plot(x_pos((k-1)*100+1:k*100),y_pos((k-1)*100+1:k*100),'DisplayName','Bezier Curves');
    f1.Color = color(k);
    scatter(poly_coef_x((k-1)*(n_order+1)+1:k*(n_order+1)),...
        poly_coef_y((k-1)*(n_order+1)+1:k*(n_order+1)),color(k));
end
% f2 = plot(q(1,:),q(2,:),'DisplayName','B-splines');
% legend([f1,f2]);
hold off

% plot p v a j seperately
idx = 1;
figure
plot(linspace(0,5,500),x_pos);
hold on
plot(linspace(0,5,500),y_pos);
hold on

function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, n_seg, n_order, v_max, a_max, j_max)
start_cond = [waypoints(1), 0, 0, 0];
end_cond   = [waypoints(end), 0, 0, 0];

%% #####################################################
% STEP 1: compute Q_0 of c'Q_0c
[Q, M]  = getQM(n_seg, n_order, ts);
Q_0 = M'*Q*M;
Q_0 = nearestSPD(Q_0);

%% #####################################################
% STEP 2: get Aeq and beq
[Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond);

%% #####################################################
% STEP 3: get corridor_range, Aieq and bieq

% STEP 3.1: get corridor_range of x-axis or y-axis,
% you can define corridor_range as [p1_min, p1_max;
%                                   p2_min, p2_max;
%                                   ...,
%                                   pn_min, pn_max];
% corridor_range = zeros(size(corridor,2),2);
% for i = 1:size(corridor,2)
%     corridor_range(i,1) = corridor(1,i) - corridor(2,i);
%     corridor_range(i,2) = corridor(1,i) + corridor(2,i);
% end
d_order = 4;
constraint_range = zeros(n_seg, 2*d_order);
for j=0:n_seg-1
    constraint_range(j+1,:)=[corridor(axis,j+1)+corridor(2+axis,j+1),...
        -(corridor(axis,j+1)-corridor(2+axis,j+1)),...
        v_max,v_max,a_max,a_max,j_max,j_max];
end

% STEP 3.2: get Aieq and bieq
[Aieq, bieq] = getAbieq(n_seg, d_order, constraint_range,ts);
f = zeros(size(Q_0,1),1);
poly_coef = quadprog(Q_0,f,Aieq, bieq, Aeq, beq);
end

function plot_rect(center, x_r, y_r)
p1 = center+[-x_r;-y_r];
p2 = center+[-x_r;y_r];
p3 = center+[x_r;y_r];
p4 = center+[x_r;-y_r];
plot_line(p1,p2);
plot_line(p2,p3);
plot_line(p3,p4);
plot_line(p4,p1);
end

function plot_line(p1,p2)
a = [p1(:),p2(:)];
plot(a(1,:),a(2,:),'b');
end