%% Problem Description
% Actions 1 = N, 2 = E, 3 = S, 4 = W
clc;
clear;
close all;
%% Initial Parameters
x0 = [0 0];         % Start State
goal = [pi/2 pi/2];    % End State
wp1 = [pi 0];
r = -0.01;           % Living Reward
rwp = 0;
R = 10;                % End Reward
gamma = 1;
nIter = 500;
eps = 0.1;
%% Obstacle Modeling
L = linspace(0,2*pi,6);
L2 = linspace(0,2*pi,5);
L3 = linspace(0,2*pi,100);
L4 = linspace(0,2*pi,6);
xv = 1.5+1*cos(L)';
yv = 1.5+1*sin(L)';
npoints = 20;

xv2 = -0.5+0.3*cos(L2)';
yv2 = 0.5+0.3*sin(L2)';
% 
xv3 = 1.4+1*cos(L3)';
yv3 = -1.5+1*sin(L3)';

xv4 = -1.4+1*cos(L4)';
yv4 = -1.3+1*sin(L4)';

% q1 = pi/4; q2 = 0;

% E1 = [cos(q1) sin(q1)];
% E2 = [cos(q1)+cos(q1 + q2) sin(q1)+sin(q1 + q2)];
% plot(points(:,1),points(:,2),'bo','Markersize',2)



grid1 = 2*pi/100;
grid2 = 2*pi/100;
th1 = -pi:grid1:pi;
th2 = -pi:grid1:pi;


states = zeros(length(th1),length(th2),2);
V0 = zeros(length(th1),length(th2));
dummy_pol = ones(length(th1),length(th2));
update1 = 0;
update2 = 0;
update3 = 0;
l1 = length(th1);
l2 = length(th2);
colijs = [];

for i = 1:length(th1)
    for j = 1:length(th2)
        states(i,j,1) = th1(i);
        states(i,j,2) = th2(j);
        if (abs(th1(i)-x0(1)) <= grid1) && (abs(th2(j)-x0(2)) <= grid2)
            if update1 == 0
                istart = i;
                jstart = j;
                update1 = 1;
            end
        end
        
        if (abs(th1(i)-goal(1)) <= grid1) && (abs(th2(j)-goal(2)) <= grid2)
            if update2 == 0
%                 V0(i,j) = R;
%                 dummy_pol(i,j) = 0;
                update2 = 1;
                iend = i;
                jend = j;
            end
        end
        
        % Collision Check
        q1 = th1(i); q2 = th2(j);
        E1 = [cos(q1) sin(q1)];
        E2 = [cos(q1)+cos(q1 + q2) sin(q1)+sin(q1 + q2)];
        xlin1 = linspace(0,E1(1),npoints);
        ylin1 = linspace(0,E1(2),npoints);
        points1 = [xlin1(:) ylin1(:)];

        xlin2 = linspace(E1(1),E2(1),npoints);
        ylin2 = linspace(E1(2),E2(2),npoints);
        points2 = [xlin2(:) ylin2(:)];

        points = vertcat(points1,points2);
        xq = points(:,1);
        yq = points(:,2);
        [in1,on1] = inpolygon(xq,yq,xv,yv);
        [in2,on2] = inpolygon(xq,yq,xv2,yv2);
        [in3,on3] = inpolygon(xq,yq,xv3,yv3);
        [in4,on4] = inpolygon(xq,yq,xv4,yv4);
        col_points = numel(xq(in1)) + numel(xq(on1))+numel(xq(in2)) + numel(xq(on2))+numel(xq(in3)) + numel(xq(on3))...
                    +numel(xq(in4)) + numel(xq(on4));
        
        if col_points ~=0
            colijs(end+1,:) = [i j];
        end
        
        
    end
end
nstates = length(th1)*length(th2);
% policy = ones(length(th1),length(th2));

% crnt_pol = policy;
% iter = 1;
% alpha = 0.5;
episode = 1;

V = V0;
U = V;
Q0 = zeros(l1,l2,8);
Q = Q0;

s = x0;
% Vold = V0;
% Vnew = V0;
% disp('Actions 1 = N, 2 = E, 3 = S, 4 = W');
% iter = 1;
epilength = 0;
while episode < nIter
    U = V;
    s1 = s(1);
    s2 = s(2);
    iref = find(th1 == s1);
    jref = find(th2 == s2);
    roll = rand(1);         % Epsilon-Greedy Roll
%     idiff = iend - iref;
%     jdiff = jend - jref;
    
%     if idiff < 0 && jdiff < 0
%         A = [1 2 4];
%     elseif idiff < 0 && jdiff > 0
%         A = [2 3 6];
%     elseif idiff > 0 && jdiff < 0
%         A = [4 7 8];
%     elseif idiff > 0 && jdiff > 0
%         A = [6 8 9];
%     elseif idiff == 0 && jdiff < 0
%         A = [1 4 7];
%     elseif idiff == 0 && jdiff > 0
%         A = [3 6 9];
%     elseif idiff < 0 && jdiff == 0
%         A = [1 2 3];
%     elseif idiff > 0 && jdiff == 0
%         A = [7 8 9];    
%     end
%     A = [1 2 3 4 6 7 8 9];    
    if roll < eps        
        a = randi(8);
    else
        [M,ind] = max(Q(iref,jref,:));
        a = ind;
    end
    
    snext = nextstate(iref,jref,a,states,colijs);

        
    fprintf('Episode: %d, ', episode);
    fprintf(' States: (%.4f, %.4f)\n',snext(1),snext(2));
    inxt = find(th1 == snext(1));
    jnxt = find(th2 == snext(2));
    if (round(s1,4) == round(goal(1),4) && round(s2,4) == round(goal(2),4))
        reward = R;
%     elseif (round(snext(1),4) == round(s1,4) && round(snext(2),4) == round(s2,4))
%         reward = 
    else 
        reward = r;
    end
    
    
    alpha = 1/episode;      % Decaying Learning Rate
    Q(iref,jref,a) = (1-alpha)*Q(iref,jref,a) + alpha*(reward + gamma*max(Q(inxt,jnxt,:)));

    if (round(snext(1),4) == round(goal(1),4) && round(snext(2),4) == round(goal(2),4))
        
        fprintf('End of Episode %d!\n', episode); 
        fprintf('Episode Length is %d!\n', epilength); 
        Q(inxt,jnxt,:) = (1-alpha)*Q(inxt,jnxt,:) + alpha*(R);
        episode = episode + 1;
        s = x0;
%         V = V0;
        for i = 1:l1
            for j = 1:l2
                [Max,I] = max(Q(i,j,:));
                V(i,j) = Max;
            end
        end
        disp('=========================================');
        epilength = 0;
    else
        s = snext;
        epilength = epilength+1;
    end
   
end 
fprintf('State Values after %d episodes!',episode)
V
Policy = zeros(length(th1),length(th2));
for i = 1:l1
    for j = 1:l2
        [Max,I] = max(Q(i,j,:));
        V(i,j) = Max;
        Policy(i,j) = I;
    end
end
% Policy(1,4) = 0;
% Policy(2,4) = 0;
% Policy(2,2) = 0;

% fprintf('Optimal Policy after %d episodes is',episode)
% Policy

pathlength = 1;
i = istart;
j = jstart;
qout = x0;
count = 0;
while pathlength > 0
    a = Policy(i,j);
    snext = nextstate(i,j,a,states,colijs);
    qout(end+1,:) = snext;
    i = find(th1 == snext(1));
    j = find(th2 == snext(2));
    
    if i == iend && j == jend
        break;
    end
    
    
end
qout(end+1,:) = goal;
mdl_planar2;
for i = 1:size(qout,1)
   p2.plot(qout(i,:))
   hold on;
   plot3(xv,yv,zeros(length(xv),1),'r','Linewidth',2);
   plot3(xv2,yv2,zeros(length(xv2),1),'r','Linewidth',2);
   plot3(xv3,yv3,zeros(length(xv3),1),'r','Linewidth',2);
   plot3(xv4,yv4,zeros(length(xv4),1),'r','Linewidth',2);
   fill3(xv,yv,zeros(length(xv),1),'r');
   fill3(xv2,yv2,zeros(length(xv2),1),'r');
   fill3(xv3,yv3,zeros(length(xv3),1),'r');
   fill3(xv4,yv4,zeros(length(xv4),1),'r');
end

function [snext] = nextstate(i,j,a,states,colijs)
nrows = size(states,1);
ncols = size(states,2);

if i == 1 && j == 1
    i1 = nrows; j1 = ncols;
    i2 = nrows; j2 = 1;
    i3 = nrows; j3 = 2;
    i4 = 1; j4 = ncols;
    i5 = 1; j5 = 1;
    i6 = 1; j6 = 2;
    i7 = 2; j7 = ncols;
    i8 = 2; j8 = 1;
    i9 = 2; j9 = 2;
elseif i == nrows && j == 1
    i1 = nrows-1; j1 = ncols;
    i2 = nrows-1; j2 = 1;
    i3 = nrows-1; j3 = 2;
    i4 = nrows; j4 = ncols;
    i5 = nrows; j5 = 1;
    i6 = nrows; j6 = 2;
    i7 = 1; j7 = ncols;
    i8 = 1; j8 = 1;
    i9 = 1; j9 = 2;
elseif i == 1 && j == ncols
    i1 = nrows; j1 = ncols-1;
    i2 = nrows; j2 = ncols;
    i3 = nrows; j3 = 1;
    i4 = 1; j4 = ncols-1;
    i5 = 1; j5 = ncols;
    i6 = 1; j6 = 1;
    i7 = 2; j7 = ncols-1;
    i8 = 2; j8 = ncols;
    i9 = 2; j9 = 1;
elseif i == nrows && j == ncols
    i1 = nrows-1; j1 = ncols-1;
    i2 = nrows-1; j2 = ncols;
    i3 = nrows-1; j3 = 1;
    i4 = nrows; j4 = ncols-1;
    i5 = nrows; j5 = ncols;
    i6 = nrows; j6 = 1;
    i7 = 1; j7 = ncols-1;
    i8 = 1; j8 = ncols;
    i9 = 1; j9 = 1;
elseif i == 1
    i1 = nrows; j1 = j-1;
    i2 = nrows; j2 = j;
    i3 = nrows; j3 = j+1;
    i4 = 1; j4 = j-1;
    i5 = 1; j5 = j;
    i6 = 1; j6 = j+1;
    i7 = 2; j7 = j-1;
    i8 = 2; j8 = j;
    i9 = 2; j9 = j+1;
elseif i == nrows
    i1 = nrows-1; j1 = j-1;
    i2 = nrows-1; j2 = j;
    i3 = nrows-1; j3 = j+1;
    i4 = nrows; j4 = j-1;
    i5 = nrows; j5 = j;
    i6 = nrows; j6 = j+1;
    i7 = 1; j7 = j-1;
    i8 = 1; j8 = j;
    i9 = 1; j9 = j+1;    
elseif j == 1
    i1 = i-1; j1 = ncols;
    i2 = i-1; j2 = j;
    i3 = i-1; j3 = j+1;
    i4 = i; j4 = ncols;
    i5 = i; j5 = j;
    i6 = i; j6 = j+1;
    i7 = i+1; j7 = ncols;
    i8 = i+1; j8 = j;
    i9 = i+1; j9 = j+1;
elseif j == ncols
    i1 = i-1; j1 = j-1;
    i2 = i-1; j2 = j;
    i3 = i-1; j3 = 1;
    i4 = i; j4 = j-1;
    i5 = i; j5 = j;
    i6 = i; j6 = 1;
    i7 = i+1; j7 = j-1;
    i8 = i+1; j8 = j;
    i9 = i+1; j9 = 1;    
else
    i1 = i-1; j1 = j-1;
    i2 = i-1; j2 = j;
    i3 = i-1; j3 = j+1;
    i4 = i; j4 = j-1;
    i5 = i; j5 = j;
    i6 = i; j6 = j+1;
    i7 = i+1; j7 = j-1;
    i8 = i+1; j8 = j;
    i9 = i+1; j9 = j+1; 
    
end

if a == 1
    for colindex = 1:size(colijs,1)
        if i1 == colijs(colindex,1) && j1 == colijs(colindex,2)
            i1 = i; j1 = j;
            break
        end            
    end
    snext = [states(i1,j1,1),states(i1,j1,2)];
elseif a == 2
    for colindex = 1:size(colijs,1)
        if i2 == colijs(colindex,1) && j2 == colijs(colindex,2)
            i2 = i; j2 = j;
            break
        end
    end
        snext = [states(i2,j2,1),states(i2,j2,2)];
elseif a == 3
    for colindex = 1:size(colijs,1)
        if i3 == colijs(colindex,1) && j3 == colijs(colindex,2)
            i3 = i; j3 = j;
            break
        end
    end
    snext = [states(i3,j3,1),states(i3,j3,2)];
elseif a == 4
    for colindex = 1:size(colijs,1)
        if i4 == colijs(colindex,1) && j4 == colijs(colindex,2)
            i4 = i; j4 = j;
            break
        end
    end
    snext = [states(i4,j4,1),states(i4,j4,2)];    
% elseif a == 5
%     for colindex = 1:size(colijs,1)
%         if i5 == colijs(colindex,1) && j5 == colijs(colindex,2)
%             i5 = i; j5 = j;
%         end
%     end
%     snext = [states(i5,j5,1),states(i5,j5,2)];
elseif a == 5
    for colindex = 1:size(colijs,1)
        if i6 == colijs(colindex,1) && j6 == colijs(colindex,2)
            i6 = i; j6 = j;
            break
        end
    end
    snext = [states(i6,j6,1),states(i6,j6,2)];    
elseif a == 6
    for colindex = 1:size(colijs,1)
        if i7 == colijs(colindex,1) && j7 == colijs(colindex,2)
            i7 = i; j7 = j;
            break
        end
    end
    snext = [states(i7,j7,1),states(i7,j7,2)];    
elseif a == 7
    for colindex = 1:size(colijs,1)
        if i8 == colijs(colindex,1) && j8 == colijs(colindex,2)
            i8 = i; j8 = j;
            break
        end
    end
    snext = [states(i8,j8,1),states(i8,j8,2)];    
elseif a == 8
    for colindex = 1:size(colijs,1)
        if i9 == colijs(colindex,1) && j9 == colijs(colindex,2)
            i9 = i; j9 = j;
            break
        end
    end
    snext = [states(i9,j9,1),states(i9,j9,2)];    
end


end
