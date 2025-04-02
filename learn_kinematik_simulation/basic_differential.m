clc;close;clear;

% parameters
rR=0.075;
rL=rR;
b=0.3/2;
tf=6;

% initial conditions
theta0=0;
x0=0;
y0=0;

% time vector and time increment
t=linspace(0,tf,120);
dt=t(2)-t(1);

% initialization and pre-allocation of pose variables
x=zeros(1,length(t));
y=zeros(1,length(t));
theta=zeros(1,length(t));
theta(1)=theta0;
x(1)=x0;
y(1)=y0;

% command signals (in RPM)
ind1=find(t<=max(t)/6);
ind2=find((t>max(t)/6)&(t<=2*max(t)/6));
ind3=find((t>2*max(t)/6)&(t<=max(t))); 

vec=ones(1,length(t));
phidotR=[240*(2*pi/60)*(vec(ind1)) 120*(2*pi/60)*(vec(ind2)) 120*(2*pi/60)*(vec(ind3)) ];
phidotL=[120*(2*pi/60)*(vec(ind1)) 240*(2*pi/60)*(vec(ind2)) -120*(2*pi/60)*(vec(ind3)) ];

% update loop
for ii=2:length(t)
    J=[rR*cos(theta(ii-1))/2 rL*cos(theta(ii-1))/2;...
        rR*sin(theta(ii-1))/2 rL*sin(theta(ii-1))/2;...
        0.5*rR/b -0.5*rL/b];

    deltapose=dt*J*[phidotR(ii-1);phidotL(ii-1)];

    x(ii)=x(ii-1)+deltapose(1);
    y(ii)=y(ii-1)+deltapose(2);
    theta(ii)=theta(ii-1)+deltapose(3);
end
    
% plot and animation
figure(1);
plot(t,phidotR,'b');
hold on;
plot(t,phidotL,'r');
xlabel('t')
legend('d(\phi_{R})/dt','d(\phi_{L})/dt')
title('angular velocities of the wheels');

figure(2);
plot(t,theta,'b');
xlabel('t')
ylabel('\theta')
title('angle of the robot chassis');

figure(3);
plot(x,y,'k--');
title('workspace')
xlabel('x')
ylabel('y')
hold on;
wls=[0 0;-0.5*b 0.5*b];
wlsrot=[cos(theta(ii)) -sin(theta(ii));sin(theta(ii)) cos(theta(ii))]*wls;
h1=plot(wlsrot(1,1)+x(1),wlsrot(2,1)+y(1),'ro','LineWidth',2,'MarkerFaceColor','r');
h2=plot(wlsrot(1,2)+x(1),wlsrot(2,2)+y(1),'ro','LineWidth',2,'MarkerFaceColor','r');
h3=plot(x(1),y(1),'bo','MarkerSize',20);
axis([-1.25 1.25 -0.5 2]);

for ii=2:length(t)
    wlsrot=[cos(theta(ii)) -sin(theta(ii));sin(theta(ii)) cos(theta(ii))]*wls;
    set(h1,'XData',wlsrot(1,1)+x(ii));
    set(h1,'YData',wlsrot(2,1)+y(ii));
    set(h2,'XData',wlsrot(1,2)+x(ii));
    set(h2,'YData',wlsrot(2,2)+y(ii));
    set(h3,'XData',x(ii));
    set(h3,'YData',y(ii));
    
    drawnow;
    pause(0.1);
end