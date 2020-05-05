%% Problem Description
% This file contains the policy iteration algorithm written to evaluate the 
% optimal path for a 2-Link Planar Robotic Arm with obstacles.
clc;
clear;
close all;
%% Initial Parameters
x0 = [0 0];                     % Start State
goal = [pi 0];                  % End State
r = -0.01;                      % Living Reward
R = 100;                        % End Reward

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
disp('Initializing the Problem...')
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

%% Creating MINUS, PLUS ARRAYS FOR WRITING TRANSITION MODEL
iminus = zeros(l1,1);
iplus = zeros(l1,1);
jminus = zeros(l2,1);
jplus = zeros(l2,1);

for i = 1:l1
    iminus(i) = i-1;
    iplus(i) = i+1;                       
    if i == 1
        iminus(i) = l1 - 1;
    elseif i == l1
        iplus(i) = 2;
    end                
end

for j = 1:l2
    jminus(j) = j-1;
    jplus(j) = j+1;                       
    if j == 1
        jminus(j) = l2 - 1;
    elseif j == l2
        jplus(j) = 2;
    end                
end
%% Initiating Few Other Parameters...
Vold = V0;
Vnew = V0;
policy = ones(length(th1),length(th2));

crnt_pol = policy;
iter = 1;
pol_iter = 1;

%% Policy Iteration
disp('Policy Iteration Starts...')
while iter >  0
    Vold = Vnew;
    Q = Qfunc(Vnew,r,l1,l2,iminus,iplus,jminus,jplus,colijs);    
    for i = 1:l1
       for j = 1:l2
          if crnt_pol(i,j) ~= 0
              Vnew(i,j) = Q(i,j,crnt_pol(i,j));
          end           
       end
    end
   
    fprintf('Value Determination Iteration: %d\n',iter);
    iter = iter + 1; 
    
    if Vold == Vnew
        Q = Qfunc(Vnew,r,l1,l2,iminus,iplus,jminus,jplus,colijs);
        Vint = V0;
        Policyint = zeros(length(th1),length(th2));
        for i = 1:l1
        for j = 1:l2
          if dummy_pol(i,j) ~= 0
              [maxval,index] = max(Q(i,j,:));
                Vint(i,j) = maxval;
                Policyint(i,j) = index;
          end           
        end
        end
        if crnt_pol == Policyint
            disp('==================================')
            Optimal_Policy = crnt_pol;
            disp('Policy Converged!')
            break
        else
            Vnew(:,:) = Vint(:,:);
            crnt_pol(:,:) = Policyint(:,:);
            disp('POLICY IMPROVED!')
            pol_iter = pol_iter + 1;
            disp('==================================')
            disp('==================================')
            iter = 1;
        end
    end        
end

disp('Policy Iteration Method Found an Optimal Policy ')
fprintf('Number of Policy Iterations: %d\n',pol_iter);

%% Retrieving the Optimal Policy

pathlength = 1;
i = istart;
j = jstart;
qout = x0;
count = 0;
while pathlength > 0
    a = Optimal_Policy(i,j);
    snext = nexts(i,j,a,states,iminus,iplus,jminus,jplus);
    qout(end+1,:) = snext;
    i = find(th1 == snext(1));
    j = find(th2 == snext(2));
    
    if i == iend && j == jend
        break;
    end
    
    
end
qout(end+1,:) = goal;

%% Visualizing the Optimal Policy
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

endpoints = [];
for i = 1:size(qout,1)-1
    endpoints(i,:) = [cos(qout(i,1))+cos(qout(i,1) + qout(i,2)) sin(qout(i,1))+sin(qout(i,1) + qout(i,2))];
    hold on;
end
plot3(endpoints(:,1),endpoints(:,2),zeros(length(endpoints),1),'k','Linewidth',2)

%% Action Value and Transition Functions are Defined Below.
% Action Value Function
function [Q] = Qfunc(V,r,l1,l2,iminus,iplus,jminus,jplus,colijs)
    Q = zeros(l1,l2,9);
    % 9 Actions
    for m = 1:length(colijs)
        V(colijs(m,1),colijs(m,2)) = 0;
    end    
    for i = 1:l1
        for j = 1:l2
                Q(i,j,1) = V(iminus(i),jminus(j))+r;
                Q(i,j,2) = V(iminus(i),j)+r;
                Q(i,j,3) = V(iminus(i),jplus(j))+r;
                Q(i,j,4) = V(i,jminus(j))+r;
                Q(i,j,5) = V(i,j)+r;
                Q(i,j,6) = V(i,jplus(j))+r;
                Q(i,j,7) = V(iplus(i),jminus(j))+r;
                Q(i,j,8) = V(iplus(i),j)+r;
                Q(i,j,9) = V(iplus(i),jplus(j))+r;
        end
    end  
end

% Transition Function
function [snext] = nexts(i,j,a,states,iminus,iplus,jminus,jplus)
ni = size(states,1);
nj = size(states,2);

Nextindex = [iminus(i) jminus(j);
             iminus(i) j;
             iminus(i) jplus(j);
             i jminus(j);
             i j;
             i jplus(j);
             iplus(i) jminus(j);
             iplus(i) j;
             iplus(i) jplus(j)];

inext = Nextindex(a,1);
jnext = Nextindex(a,2);

snext = states(inext,jnext,:); 
    
end
