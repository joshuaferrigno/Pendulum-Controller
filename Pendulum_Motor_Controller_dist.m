% Joshua Ferrigno
% Department of Mechanical and Aerospace Engineering - UT Arlington

clear all
close all
clc

l=5; %length of pendulum [meters]
init=pi; %initial angle
g=9.81; %gravity meters/second^2
t0=0; %initial time
tf=70; %final time
[t,y]=ode45(@angle,[t0:.1:tf],[init 0]);

%graph
set(gca,'nextplot','replacechildren'); 
v = VideoWriter('Pendulum_Motor_Controller_Final.avi');
v.Quality = 100;
v.FrameRate = 60;
open(v);
for i=1:size([t0:.1:tf],2)
    hold off
    plot([0 2*l*sin(y(1+i,1))],[0 2*l*cos(y(1+i,1))],'-b','LineWidth',7.5)
    hold on
    grid on

    plot(2*l*sin(y(1+i,1)),2*l*cos(y(1+i,1)),'.b','MarkerSize',100)
    title('Pendulum controller w motor input')
    axis([-2.15*l 2.15*l -2.15*l 2.15*l])   
    xlabel('X')
    ylabel('Y')
    q = getframe(gcf);
    writeVideo(v,q);
    hold on

end
close(v); 

function dthdt=angle(t,th)
l=5; %length of pendulum [meters]
m=5; %mass of pendulum [kg]
g=9.81; %gravity [meters/second^2]
thetawrap=wrapToPi(th(1));
thetad=0;
thetad=wrapToPi(thetad);
E=thetawrap-thetad;
kd=.3;
kp=255;
T=kp*(-E+kd*(-th(2)));
dthdt=[th(2);(g*m*l*sin(th(1))+T)/(m*l^2)];
end   