%% Problem Description
% Actions 1 = N, 2 = E, 3 = S, 4 = W
clc;
clear;
close all;
%% Initial Parameters
x0 = [0 0];         % Start State
goal = [pi/2 pi/2];    % End State
R = 100;                % End Reward
nIter = 1000;
r = -0.1;
gamma = 1;
eps = 0.1;

grid1 = 2*pi/100;
grid2 = 2*pi/100;
th1 = -pi:grid1:pi;
th2 = -pi:grid1:pi;
l1 = length(th1);
l2 = length(th2);
states = zeros(l1,l2,2);
update1 = 0;
update2 = 0;
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
                update2 = 1;
                iend = i;
                jend = j;
            end
        end
    end
end


V0 = zeros(l1,l2);
Q0 = zeros(l1,l2,8);
Q = Q0;
episode = 1;
% alpha = 0.5;

V = V0;
U = V;

s0 = x0;

s = s0;
% Vold = V0;
% Vnew = V0;
% disp('Actions 1 = N, 2 = E, 3 = S, 4 = W');
% iter = 1;
while episode < nIter
    U = V;
    s1 = s(1);
    s2 = s(2);
    icrnt = find(th1 == s1);
    jcrnt = find(th2 == s2);
    roll = rand(1);         % Epsilon-Greedy Roll
%     eps = 1/episode;      % Decaying Learning Rate
    if roll < eps        
        a = randi(8);
    else
        [M,ind] = max(Q(icrnt,jcrnt,:));
        a = ind;
    end
    
    snext = nextstate(icrnt,jcrnt,a,states);
    inxt = find(th1 == snext(1));
    jnxt = find(th2 == snext(2));
    if (round(s1,4) == round(goal(1),4) && round(s2,4) == round(goal(2),4))
        reward = R;
    else
        reward = r;
    end
    alpha = 1/episode;      % Decaying Learning Rate
    Q(icrnt,jcrnt,a) = (1-alpha)*Q(icrnt,jcrnt,a) + alpha*(reward + gamma*max(Q(inxt,jnxt,:)));
    fprintf('Episode: %d', episode);
    fprintf('Current State: (%.4f, %.4f)\n',snext(1),snext(2));
    if (round(snext(1),4) == round(goal(1),4) && round(snext(2),4) == round(goal(2),4))
        fprintf('End of Episode %d!\n', episode);        
        Q(inxt,jnxt,:) = (1-alpha)*Q(inxt,jnxt,:) + alpha*(R);
        episode = episode + 1;
        s = s0;
        V = V0;
        for i = 1:l1
            for j = 1:l2
                [Max,I] = max(Q(i,j,:));
                V(i,j) = Max;
            end
        end
        disp('=========================================');
    else
        s = snext;
    end
   
end 
fprintf('State Values after %d episodes!',episode)
V
Optimal_Policy = zeros(l1,l2);
for i = 1:l1
    for j = 1:l2
        [Max,I] = max(Q(i,j,:));
        V(i,j) = Max;
        Optimal_Policy(i,j) = I;
    end
end
Optimal_Policy(iend,jend) = 0;

pathlength = 1;
i = istart;
j = jstart;
qout = x0;
count = 0;
while pathlength > 0
    a = Optimal_Policy(i,j);
    snext = nextstate(i,j,a,states);
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
%    plot3(xv,yv,zeros(length(xv),1),'r','Linewidth',2);
%    plot3(xv2,yv2,zeros(length(xv2),1),'r','Linewidth',2);
%    plot3(xv3,yv3,zeros(length(xv3),1),'r','Linewidth',2);
%    plot3(xv4,yv4,zeros(length(xv4),1),'r','Linewidth',2);
%    fill3(xv,yv,zeros(length(xv),1),'r');
%    fill3(xv2,yv2,zeros(length(xv2),1),'r');
%    fill3(xv3,yv3,zeros(length(xv3),1),'r');
%    fill3(xv4,yv4,zeros(length(xv4),1),'r');
end



function [snext] = nextstate(i,j,a,states)
nrows = size(states,1);
ncols = size(states,2);

if i == 1 && j == 1
    i1 = nrows; j1 = ncols;
    i2 = nrows; j2 = 1;
    i3 = nrows; j3 = 2;
    i4 = 1; j4 = ncols;
    i6 = 1; j6 = 2;
    i7 = 2; j7 = ncols;
    i8 = 2; j8 = 1;
    i9 = 2; j9 = 2;
elseif i == nrows && j == 1
    i1 = nrows-1; j1 = ncols;
    i2 = nrows-1; j2 = 1;
    i3 = nrows-1; j3 = 2;
    i4 = nrows; j4 = ncols;
    i6 = nrows; j6 = 2;
    i7 = 1; j7 = ncols;
    i8 = 1; j8 = 1;
    i9 = 1; j9 = 2;
elseif i == 1 && j == ncols
    i1 = nrows; j1 = ncols-1;
    i2 = nrows; j2 = ncols;
    i3 = nrows; j3 = 1;
    i4 = 1; j4 = ncols-1;
    i6 = 1; j6 = 1;
    i7 = 2; j7 = ncols-1;
    i8 = 2; j8 = ncols;
    i9 = 2; j9 = 1;
elseif i == nrows && j == ncols
    i1 = nrows-1; j1 = ncols-1;
    i2 = nrows-1; j2 = ncols;
    i3 = nrows-1; j3 = 1;
    i4 = nrows; j4 = ncols-1;
    i6 = nrows; j6 = 1;
    i7 = 1; j7 = ncols-1;
    i8 = 1; j8 = ncols;
    i9 = 1; j9 = 1;
elseif i == 1
    i1 = nrows; j1 = j-1;
    i2 = nrows; j2 = j;
    i3 = nrows; j3 = j+1;
    i4 = 1; j4 = j-1;
    i6 = 1; j6 = j+1;
    i7 = 2; j7 = j-1;
    i8 = 2; j8 = j;
    i9 = 2; j9 = j+1;
elseif i == nrows
    i1 = nrows-1; j1 = j-1;
    i2 = nrows-1; j2 = j;
    i3 = nrows-1; j3 = j+1;
    i4 = nrows; j4 = j-1;
    i6 = nrows; j6 = j+1;
    i7 = 1; j7 = j-1;
    i8 = 1; j8 = j;
    i9 = 1; j9 = j+1;    
elseif j == 1
    i1 = i-1; j1 = ncols;
    i2 = i-1; j2 = j;
    i3 = i-1; j3 = j+1;
    i4 = i; j4 = ncols;
    i6 = i; j6 = j+1;
    i7 = i+1; j7 = ncols;
    i8 = i+1; j8 = j;
    i9 = i+1; j9 = j+1;
elseif j == ncols
    i1 = i-1; j1 = j-1;
    i2 = i-1; j2 = j;
    i3 = i-1; j3 = 1;
    i4 = i; j4 = j-1;
    i6 = i; j6 = 1;
    i7 = i+1; j7 = j-1;
    i8 = i+1; j8 = j;
    i9 = i+1; j9 = 1;    
else
    i1 = i-1; j1 = j-1;
    i2 = i-1; j2 = j;
    i3 = i-1; j3 = j+1;
    i4 = i; j4 = j-1;
    i6 = i; j6 = j+1;
    i7 = i+1; j7 = j-1;
    i8 = i+1; j8 = j;
    i9 = i+1; j9 = j+1; 
    
end

if a == 1
    snext = [states(i1,j1,1),states(i1,j1,2)];
elseif a == 2
    snext = [states(i2,j2,1),states(i2,j2,2)];
elseif a == 3
    snext = [states(i3,j3,1),states(i3,j3,2)];
elseif a == 4
    snext = [states(i4,j4,1),states(i4,j4,2)];    
elseif a == 5
    snext = [states(i6,j6,1),states(i6,j6,2)];    
elseif a == 6
    snext = [states(i7,j7,1),states(i7,j7,2)];    
elseif a == 7
    snext = [states(i8,j8,1),states(i8,j8,2)];    
elseif a == 8
    snext = [states(i9,j9,1),states(i9,j9,2)];    
end     
end
