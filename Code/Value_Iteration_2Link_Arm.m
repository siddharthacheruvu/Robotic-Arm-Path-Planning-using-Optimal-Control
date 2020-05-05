%% Problem Description
% This file contains the value iteration algorithm written to evaluate the 
% optimal path for a 2-Link Planar Robotic Arm without any obstacles.
clc;
clear;
close all;
%% Parameters  
% (User can Modify with the Start and Goal States to check how the results vary!)
% ----Start State----
x0 = [0 0];
% ----End State-----
goal = [7*pi/8 -pi/4];      
r = -0.2;                   % Living Reward / Penalty
R = 10;                     % End Reward

grid1 = 2*pi/100;           % Grid Size of theta1
grid2 = 2*pi/100;           % Grid Size of theta2
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

%% Identifying the start and end joint angle indices
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
    end
end
nstates = l1*l2;

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
iter = 1;
policy = zeros(length(th1),length(th2));

%% Value Iteration
while iter >  0
    Vold = Vnew;
    Q = Qfunc(Vnew,r,l1,l2,iminus,iplus,jminus,jplus);        % Q value calculation using Bellman's Equation
    for i = 1:l1
       for j = 1:l2
          if dummy_pol(i,j) ~= 0
              [maxval,index] = max(Q(i,j,:));
                Vnew(i,j) = maxval;
                policy(i,j) = index;
          end           
       end
    end
   
    fprintf('Value Iteration: %d\n',iter);
    iter = iter + 1; 
    
    if Vold == Vnew             % Convergence Check
        disp('==================================')    
        disp('Values Converged!')
        break
    end        
end

fprintf('Number of value iterations performed is %d\n',iter)

%% Retrieving the Optimal Path
pathlength = 1;
i = istart;
j = jstart;
qout = x0;
while pathlength > 0
    a = policy(i,j);
    snext = nexts(i,j,a,states,iminus,iplus,jminus,jplus);
    qout(end+1,:) = snext;
    i = find(th1 == snext(1));
    j = find(th1 == snext(2));
    
    if i == iend && j == jend
        break;
    end
    
end
qout(end+1,:) = goal;

%% Visualizing the Optimal Path
mdl_planar2;
for i = 1:size(qout,1)
   p2.plot(qout(i,:))
end

endpoints = [];
for i = 1:size(qout,1)-1
    endpoints(i,:) = [cos(qout(i,1))+cos(qout(i,1) + qout(i,2)) sin(qout(i,1))+sin(qout(i,1) + qout(i,2))];
    hold on;
end
plot3(endpoints(:,1),endpoints(:,2),zeros(length(endpoints),1),'k','Linewidth',2)

%% Action Value and Transition Functions are Defined Below.
% Action Value Function
function [Q] = Qfunc(V,r,l1,l2,iminus,iplus,jminus,jplus)
    Q = zeros(l1,l2,9);
    % 9 Actions
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
