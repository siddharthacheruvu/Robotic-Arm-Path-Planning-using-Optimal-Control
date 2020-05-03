%% Problem Description
clc;
clear;
close all;
%% Initial Parameters
x0 = [0 0];         % Start State
goal = [0.8*pi/2 pi/10];    % End State
wp1 = [pi 0];
r = -0.04;           % Living Reward
rwp = 0;
R = 100;                % End Reward


%% Obstacle Modeling
L = linspace(0,2*pi,6);
xv = 1.5+1*cos(L)';
yv = 1.5+1*sin(L)';
npoints = 20;

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
                V0(i,j) = R;
                dummy_pol(i,j) = 0;
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
        [in,on] = inpolygon(xq,yq,xv,yv);
        numel(xq(in));
        numel(xq(on));

        col_points = numel(xq(in)) + numel(xq(on));
        
        if col_points ~=0
            colijs(end+1,:) = [i j];
        end
        
        
    end
end
nstates = length(th1)*length(th2);

Vold = V0;
Vnew = V0;
% disp('Actions 1 = N, 2 = E, 3 = S, 4 = W');
iter = 1;
% dummy_pol = [2 2 2 0;       % This policy is not used in value iteration.
%           1 0 3 0;          % It is just used for indexing purposes.
%           1 4 4 4];
policy = zeros(length(th1),length(th2));
while iter >  0
    Vold = Vnew;
    Q = Qfunc(Vnew,r,l1,l2,colijs);    

    for i = 1:l1
       for j = 1:l2
          if dummy_pol(i,j) ~= 0
              [maxval,index] = max(Q(i,j,:));
                Vnew(i,j) = maxval;
                policy(i,j) = index;
                targeti = i;
                targetj = j;
          end           
       end
    end
   
    fprintf('Iteration is %d\n',iter);
%     disp('The current state value matrix is ');
%     Vnew
    iter = iter + 1; 
    
    if Vold == Vnew
        disp('==================================')    
        disp('Values Converged!')
%         Vnew
%         disp('Optimal Policy is')
%         policy
%         disp('where,')
%         disp('Actions 1 = N, 2 = E, 3 = S, 4 = W');
        break
    end        
end

fprintf('Number of value iterations performed is %d\n',iter)

pathlength = 1;
i = istart;
j = jstart;
qout = x0;

while pathlength > 0
    a = policy(i,j);
    snext = trans(i,j,a,states);
    qout(end+1,:) = snext;
    i = find(th1 == snext(1));
    j = find(th1 == snext(2));
    
    if i == iend && j == jend
        break;
    end
    
end
qout(end+1,:) = goal;
mdl_planar2;
for j = 1:4
for i = 1:size(qout,1)
   p2.plot(qout(i,:))
   hold on;
   plot3(xv,yv,zeros(length(xv),1),'r','Linewidth',2);
   fill3(xv,yv,zeros(length(xv),1),'r');
end
end

%% Action Value and Transition Functions are Defined Below.

function [Q] = Qfunc(V,r,l1,l2,colijs)
    Q = zeros(l1,l2,9);
    % 9 Actions
    for i = 1:l1
        for j = 1:l2
            for k = 1:length(colijs)
               if i == colijs(k,1) && j == colijs(k,2)
                   V(i,j) = 0;
               end
                
            end            
        end
    end
    
    for i = 1
        for j = 2:l2-1
           Q(i,j,1) = V(end,j-1)+r;
           Q(i,j,2) = V(end,j)+r;
           Q(i,j,3) = V(end,j+1)+r;
           Q(i,j,4) = V(i,j-1)+r;
           Q(i,j,5) = V(i,j)+r;
           Q(i,j,6) = V(i,j+1)+r;
           Q(i,j,7) = V(i+1,j-1)+r;
           Q(i,j,8) = V(i+1,j)+r;
           Q(i,j,9) = V(i+1,j+1)+r;
        end
    end
    
    for i = l1
        for j = 2:l2-1
           Q(i,j,7) = V(1,j-1)+r;
           Q(i,j,8) = V(1,j)+r;
           Q(i,j,9) = V(1,j+1)+r;
           Q(i,j,1) = V(i-1,j-1)+r;
           Q(i,j,2) = V(i-1,j)+r;
           Q(i,j,3) = V(i-1,j+1)+r;
           Q(i,j,4) = V(i,j-1)+r;
           Q(i,j,5) = V(i,j)+r;
           Q(i,j,6) = V(i,j+1)+r;
        end
    end
    
    for i = 2:l1-1
        for j = 1
           Q(i,j,1) = V(i-1,end)+r;
           Q(i,j,4) = V(i,end)+r;
           Q(i,j,7) = V(i+1,end)+r;   
           Q(i,j,2) = V(i-1,j)+r;
           Q(i,j,3) = V(i-1,j+1)+r;
           Q(i,j,5) = V(i,j)+r;
           Q(i,j,6) = V(i,j+1)+r;
           Q(i,j,8) = V(i+1,j)+r;
           Q(i,j,9) = V(i+1,j+1)+r;          
        end
    end
    
    for i = 2:l1-1
        for j = l2
           Q(i,j,3) = V(i-1,1)+r;
           Q(i,j,6) = V(i,1)+r;
           Q(i,j,9) = V(i+1,1)+r;   
           Q(i,j,2) = V(i-1,j)+r;
           Q(i,j,1) = V(i-1,j-1)+r;
           Q(i,j,5) = V(i,j)+r;
           Q(i,j,4) = V(i,j-1)+r;
           Q(i,j,8) = V(i+1,j)+r;
           Q(i,j,7) = V(i+1,j-1)+r;          
        end
    end
           Q(1,1,1) = V(end,end)+r;
           Q(1,1,2) = V(end,1)+r;
           Q(1,1,3) = V(end,2)+r;
           Q(1,1,4) = V(1,end)+r;
           Q(1,1,7) = V(2,end)+r;
           Q(1,1,5) = V(1,1)+r;
           Q(1,1,6) = V(1,2)+r;
           Q(1,1,8) = V(2,1)+r;
           Q(1,1,9) = V(2,2)+r;
           
           Q(1,end,1) = V(end,end-1)+r;
           Q(1,end,2) = V(end,end)+r;
           Q(1,end,3) = V(end,1)+r;
           Q(1,end,6) = V(1,1)+r;
           Q(1,end,9) = V(2,1)+r;
           Q(1,end,4) = V(1,end-1)+r;
           Q(1,end,7) = V(2,end-1)+r;
           Q(1,end,5) = V(1,end)+r;           
           Q(1,end,8) = V(2,end)+r;
           
           Q(end,1,1) = V(end-1,end)+r;
           Q(end,1,2) = V(end-1,1)+r;
           Q(end,1,3) = V(end-1,2)+r;
           Q(end,1,4) = V(end,end)+r;
           Q(end,1,5) = V(end,1)+r;
           Q(end,1,6) = V(end,2)+r;
           Q(end,1,7) = V(1,end)+r;
           Q(end,1,8) = V(1,1)+r;
           Q(end,1,9) = V(1,2)+r;
           
           Q(end,end,1) = V(end-1,end-1)+r;
           Q(end,end,2) = V(end-1,end)+r;
           Q(end,end,3) = V(end-1,1)+r;
           Q(end,end,4) = V(end,end-1)+r;
           Q(end,end,5) = V(end,end)+r;
           Q(end,end,6) = V(end,1)+r;
           Q(end,end,7) = V(1,end-1)+r;
           Q(end,end,8) = V(1,end)+r;
           Q(end,end,9) = V(1,1)+r;
           
      for i = 2:l1-1
        for j = 2:l2-1
           Q(i,j,1) = V(i-1,j-1)+r;
           Q(i,j,2) = V(i-1,j)+r;
           Q(i,j,3) = V(i-1,j+1)+r;
           Q(i,j,4) = V(i,j-1)+r;
           Q(i,j,5) = V(i,j)+r;
           Q(i,j,6) = V(i,j+1)+r;
           Q(i,j,7) = V(i+1,j-1)+r;
           Q(i,j,8) = V(i+1,j)+r;
           Q(i,j,9) = V(i+1,j+1)+r;
        end
      end   
end

function [snext] = trans(i,j,a,states)

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
    snext = [states(i1,j1,1),states(i1,j1,2)];
elseif a == 2
    snext = [states(i2,j2,1),states(i2,j2,2)];
elseif a == 3
    snext = [states(i3,j3,1),states(i3,j3,2)];
elseif a == 4
    snext = [states(i4,j4,1),states(i4,j4,2)];    
elseif a == 5
    snext = [states(i5,j5,1),states(i5,j5,2)];
elseif a == 6
    snext = [states(i6,j6,1),states(i6,j6,2)];    
elseif a == 7
    snext = [states(i7,j7,1),states(i7,j7,2)];    
elseif a == 8
    snext = [states(i8,j8,1),states(i8,j8,2)];    
elseif a == 9
    snext = [states(i9,j9,1),states(i9,j9,2)];    
end   
    
end
