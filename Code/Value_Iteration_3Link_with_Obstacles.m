%% Problem Description
clc;
clear;
close all;
%% Initial Parameters
x0 = [0 0 0];         % Start State
goal = [pi 0 0];    % End State
% wp1 = [pi 0];
r = -0.2;           % Living Reward
% rwp = 0;
R = 100;                % End Reward

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
yv4 = -1.5+1*sin(L4)';

%%

grid1 = 2*pi/50;
grid2 = 2*pi/50;
grid3 = 2*pi/50;
th1 = -pi:grid1:pi;
th2 = -pi:grid1:pi;
th3 = -pi:grid1:pi;


states = zeros(length(th1),length(th2),length(th3),3);
V0 = zeros(length(th1),length(th2),length(th3));
dummy_pol = ones(length(th1),length(th2),length(th3));
update1 = 0;
update2 = 0;
update3 = 0;
l1 = length(th1);
l2 = length(th2);
l3 = length(th3);
colijs = [];
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
        
        % Collision Check
        q1 = th1(i); q2 = th2(j); q3 = th3(k);
        E1 = [cos(q1) sin(q1)];
        E2 = [cos(q1)+cos(q1 + q2) sin(q1)+sin(q1 + q2)];
        E3 = [cos(q1)+cos(q1 + q2)+cos(q1 + q2 + q3) sin(q1)+sin(q1 + q2)+sin(q1 + q2 + q3)];
        xlin1 = linspace(0,E1(1),npoints);
        ylin1 = linspace(0,E1(2),npoints);
        points1 = [xlin1(:) ylin1(:)];

        xlin2 = linspace(E1(1),E2(1),npoints);
        ylin2 = linspace(E1(2),E2(2),npoints);
        points2 = [xlin2(:) ylin2(:)];

        xlin3 = linspace(E2(1),E3(1),npoints);
        ylin3 = linspace(E2(2),E3(2),npoints);
        points3 = [xlin3(:) ylin3(:)];    
        
        points = vertcat(points1,points2,points3);
        xq = points(:,1);
        yq = points(:,2);
        [in1,on1] = inpolygon(xq,yq,xv,yv);
        [in2,on2] = inpolygon(xq,yq,xv2,yv2);
        [in3,on3] = inpolygon(xq,yq,xv3,yv3);
        [in4,on4] = inpolygon(xq,yq,xv4,yv4);
        col_points = numel(xq(in1)) + numel(xq(on1))+numel(xq(in2)) + numel(xq(on2))+numel(xq(in3)) + numel(xq(on3))...
                    +numel(xq(in4)) + numel(xq(on4));
        
        if col_points ~=0
            colijs(end+1,:) = [i j k];
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

%%


Vold = V0;
Vnew = V0;
% disp('Actions 1 = N, 2 = E, 3 = S, 4 = W');
iter = 1;
% dummy_pol = [2 2 2 0;       % This policy is not used in value iteration.
%           1 0 3 0;          % It is just used for indexing purposes.
%           1 4 4 4];
policy = zeros(length(th1),length(th2),length(th3));
while iter >  0
    Vold = Vnew;
    Q = Qfunc(Vnew,r,l1,l2,l3,iminus,iplus,jminus,jplus,kminus,kplus,colijs);    

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
mdl_planar3;
for i = 1:size(qout,1)
   p3.plot(qout(i,:))
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

%% Action Value and Transition Functions are Defined Below.

function [Q] = Qfunc(V,r,l1,l2,l3,iminus,iplus,jminus,jplus,kminus,kplus,colijs)
    Q = zeros(l1,l2,l3,27);
    % 27 Actions
    for m = 1:length(colijs)
            V(colijs(m,1),colijs(m,2),colijs(m,3)) = 0;
    end

    
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
