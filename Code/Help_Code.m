L = linspace(0,2*pi,5);
xv = -1+0.1*cos(L)';
yv = -1+0.1*sin(L)';


q1 = -5*pi/6; q2 = pi/3;
npoints = 20;
E1 = [cos(q1) sin(q1)];
E2 = [cos(q1)+cos(q1 + q2) sin(q1)+sin(q1 + q2)];

xlin1 = linspace(0,E1(1),npoints);
ylin1 = linspace(0,E1(2),npoints);
points1 = [xlin1(:) ylin1(:)]

xlin2 = linspace(E1(1),E2(1),npoints);
ylin2 = linspace(E1(2),E2(2),npoints);
points2 = [xlin2(:) ylin2(:)]

points = vertcat(points1,points2);
xq = points(:,1);
yq = points(:,2);
[in,on] = inpolygon(xq,yq,xv,yv);
numel(xq(in))
numel(xq(on))

figure

plot(xv,yv) % polygon
axis equal

hold on
plot(xq(in),yq(in),'r+') % points inside
plot(xq(~in),yq(~in),'bo') % points outside
hold off
col_points = numel(xq(in)) + numel(xq(on))
% plot(points(:,1),points(:,2),'bo','Markersize',2)