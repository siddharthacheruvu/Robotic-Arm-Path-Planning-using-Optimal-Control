%% Problem Description
% This file contains the value iteration algorithm written to evaluate the 
% optimal path for a 3-Link Planar Robotic Arm without any obstacles.
clc;
clear;
close all;
%% Initial Parameters
x0 = [0 0 0];                   % Start State
goal = [5*pi/6 pi/2 pi/4];    % End State
r = -0.2;                       % Living Reward
R = 10;                         % End Reward

grid1 = 2*pi/50;
grid2 = 2*pi/50;
grid3 = 2*pi/50;
th1 = -pi:grid1:pi;
th2 = -pi:grid2:pi;
th3 = -pi:grid3:pi;


states = zeros(length(th1),length(th2),length(th3),3);
V0 = zeros(length(th1),length(th2),length(th3));
dummy_pol = ones(length(th1),length(th2),length(th3));
update1 = 0;
update2 = 0;
update3 = 0;
l1 = length(th1);
l2 = length(th2);
l3 = length(th3);
for i = 1:length(th1)
    for j = 1:length(th2)
        for k = 1:length(th3)
        states(i,j,k,1) = th1(i);
        states(i,j,k,2) = th2(j);
        states(i,j,k,3) = th3(k);
        if (abs(th1(i)-x0(1)) <= grid1) && (abs(th2(j)-x0(2)) <= grid2) && (abs(th3(k)-x0(3)) <= grid3)
            if update1 == 0
                istart = i;
                jstart = j;
                kstart = k;
                update1 = 1;
            end
        end
        
        if (abs(th1(i)-goal(1)) <= grid1) && (abs(th2(j)-goal(2)) <= grid2) && (abs(th3(k)-goal(3)) <= grid3)
            if update2 == 0
                V0(i,j,k) = R;
                dummy_pol(i,j,k) = 0;
                update2 = 1;
                iend = i;
                jend = j;
                kend = k;
            end
        end
        
        end
    end
end
nstates = l1*l2*l3;

%% Creating MINUS, PLUS ARRAYS
iminus = zeros(l1,1);
iplus = zeros(l1,1);
jminus = zeros(l2,1);
jplus = zeros(l2,1);
kminus = zeros(l3,1);
kplus = zeros(l3,1);

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

for k = 1:l3
    kminus(k) = k-1;
    kplus(k) = k+1;                       
    if k == 1
        kminus(k) = l3 - 1;
    elseif k == l3
        kplus(k) = 2;
    end                
end

%% Initiating Few Other Parameters....
Vold = V0;
Vnew = V0;
iter = 1;
policy = zeros(length(th1),length(th2),length(th3));

%% Value Iteration
while iter >  0
    Vold = Vnew;
    Q = Qfunc(Vnew,r,l1,l2,l3,iminus,iplus,jminus,jplus,kminus,kplus);    

    for i = 1:l1
       for j = 1:l2
           for k = 1:l3
              if dummy_pol(i,j,k) ~= 0
                  [maxval,index] = max(Q(i,j,k,:));
                    Vnew(i,j,k) = maxval;
                    policy(i,j,k) = index;
                    targeti = i;
                    targetj = j;
                    targetk = k;
              end
          end           
       end
    end
   
    fprintf('Iteration is %d\n',iter);
    iter = iter + 1; 
    
    if Vold == Vnew
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
k = kstart;
qout = x0;

while pathlength > 0
    a = policy(i,j,k);
    snext = nexts(i,j,k,a,states,iminus,iplus,jminus,jplus,kminus,kplus);
    qout(end+1,:) = snext;
    i = find(th1 == snext(1));
    j = find(th2 == snext(2));
    k = find(th3 == snext(3));
    
    if i == iend && j == jend && k == kend
        break;
    end
    
end
qout(end+1,:) = goal;

%% Visualizing the Optimal Path
mdl_planar3;
for i = 1:size(qout,1)
   p3.plot(qout(i,:))
end

endpoints = [];
for i = 1:size(qout,1)-1
    endpoints(i,:) = [cos(qout(i,1))+cos(qout(i,1) + qout(i,2))+cos(qout(i,1) + qout(i,2)+qout(i,3)) sin(qout(i,1))+sin(qout(i,1) + qout(i,2))+sin(qout(i,1) + qout(i,2)+qout(i,3))];
    hold on;
end
plot3(endpoints(:,1),endpoints(:,2),zeros(length(endpoints),1),'k','Linewidth',2)

%% Action Value and Transition Functions are Defined Below.
% Action-Value Function
function [Q] = Qfunc(V,r,l1,l2,l3,iminus,iplus,jminus,jplus,kminus,kplus)
    Q = zeros(l1,l2,l3,27);
    % 27 Actions    
    for i = 1:l1
        for j = 1:l2
            for k = 1:l3
                Q(i,j,k,1) = V(iminus(i),jminus(j),k)+r;
                Q(i,j,k,2) = V(iminus(i),j,k)+r;
                Q(i,j,k,3) = V(iminus(i),jplus(j),k)+r;
                Q(i,j,k,4) = V(i,jminus(j),k)+r;
                Q(i,j,k,5) = V(i,jminus(j),k)+r;
                Q(i,j,k,6) = V(i,jplus(j),k)+r;
                Q(i,j,k,7) = V(iplus(i),jminus(j),k)+r;
                Q(i,j,k,8) = V(iplus(i),j,k)+r;
                Q(i,j,k,9) = V(iplus(i),jplus(j),k)+r;
                
                Q(i,j,k,10) = V(iminus(i),jminus(j),kminus(k))+r;
                Q(i,j,k,11) = V(iminus(i),j,kminus(k))+r;
                Q(i,j,k,12) = V(iminus(i),jplus(j),kminus(k))+r;
                Q(i,j,k,13) = V(i,jminus(j),kminus(k))+r;
                Q(i,j,k,14) = V(i,j,kminus(k))+r;
                Q(i,j,k,15) = V(i,jplus(j),kminus(k))+r;
                Q(i,j,k,16) = V(iplus(i),jminus(j),kminus(k))+r;
                Q(i,j,k,17) = V(iplus(i),j,kminus(k))+r;
                Q(i,j,k,18) = V(iplus(i),jplus(j),kminus(k))+r;
                
                Q(i,j,k,19) = V(iminus(i),jminus(j),kplus(k))+r;
                Q(i,j,k,20) = V(iminus(i),j,kplus(k))+r;
                Q(i,j,k,21) = V(iminus(i),jplus(j),kplus(k))+r;
                Q(i,j,k,22) = V(i,jminus(j),kplus(k))+r;
                Q(i,j,k,23) = V(i,j,kplus(k))+r;
                Q(i,j,k,24) = V(i,jplus(j),kplus(k))+r;
                Q(i,j,k,25) = V(iplus(i),jminus(j),kplus(k))+r;
                Q(i,j,k,26) = V(iplus(i),j,kplus(k))+r;
                Q(i,j,k,27) = V(iplus(i),jplus(j),kplus(k))+r;              
            end
        end
    end
end

% Transition Function
function [snext] = nexts(i,j,k,a,states,iminus,iplus,jminus,jplus,kminus,kplus)
ni = size(states,1);
nj = size(states,2);
nk = size(states,3);

Nextindex = [iminus(i) jminus(j) k;
             iminus(i) j k;
             iminus(i) jplus(j) k;
             i jminus(j) k;
             i jminus(j) k;
             i jplus(j) k;
             iplus(i) jminus(j) k;
             iplus(i) j k;
             iplus(i) jplus(j) k;             
             iminus(i) jminus(j) kminus(k);
             iminus(i) j kminus(k);
             iminus(i) jplus(j) kminus(k);
             i jminus(j) kminus(k);
             i j kminus(k);
             i jplus(j) kminus(k);
             iplus(i) jminus(j) kminus(k);
             iplus(i) j kminus(k);
             iplus(i) jplus(j) kminus(k);             
             iminus(i) jminus(j) kplus(k);
             iminus(i) j kplus(k);
             iminus(i) jplus(j) kplus(k);
             i jminus(j) kplus(k);
             i j kplus(k);
             i jplus(j) kplus(k);
             iplus(i) jminus(j) kplus(k);
             iplus(i) j kplus(k);
             iplus(i) jplus(j) kplus(k)];

inext = Nextindex(a,1);
jnext = Nextindex(a,2);
knext = Nextindex(a,3);

snext = states(inext,jnext,knext,:);                       
end
