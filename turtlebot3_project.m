
% PID program for a mobile robot (lemniscate and others)



% IP address of the turtlebot3
ipadress = 'http://10.10.234.54:11311'; 
rosinit(ipadress)

rostopic list 


close all

% handles for position and velocity
odom = rossubscriber('/odom'); % position
robot = rospublisher('/cmd_vel') ;
velmsg = rosmessage(robot);

res = rospublisher('/reset'); 
resmsg = rosmessage(res);
send(res, resmsg);

close all
%% PID parameters for lemniscate and others: Kp, Ki, Kd

% Parameters for distance (euclidean)
dt = 0.05; 

Kp = 0.75
Ki = 0.01;
Kd = 0;

% Parameters for orientation
Kpo = 0.03;
Kio = 0.01;
Kdo = 0; 

% initial parameters for PID (for integration and derivative)
PE = 0; IE = 0;
PEo = 0; IEo = 0;

X= []; Y = []; XR= []; YR = []; Th = []; Vh = []; Wh = [];

T = 7 % simulation time

% while true % Loop
for t = 0:dt:T
      
    
    odomdata = receive(odom, 7); % timout is 3s 
    pose = odomdata.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
    z = pose.Position.Z;
    X = [X x]; Y = [Y y]; % actual robot position
    
    % get orientation
    quat = pose.Orientation;
    angles = quat2eul( [quat.W quat.X quat.Y quat.Z] );
    theta = angles(1); % in radians    
    Th(end+1) = theta; 
    
      
% LINE
  % xr = t; yr = 0.5*t ;
  % lemniscate
%   A=1.5;
%   xr= A*cos(t)./(1+(sin(t)).^2); yr= A*(sin(t).*cos(t))./(1+(sin(t)).^2)
    
    % desired position: sin(t) function
%     R=0.7;  
%     xr =0.2*t; yr = R*sin(t);

    %desired position: circle (radius = 1)
    R = 0.5;
    xr = R*cos(t); yr = R*sin(t);

    % other desired position: complex
%     A=0.05;
%      xr = A*16*(sin(t)).^3; yr = (13*cos(t)-5*cos(3*t)-2*cos(3*t)-cos(4*t))*A;
%         xr =     0.5* t .* sin( pi * .872*sin(t)./t);
%         yr = -abs(t) .* cos(pi * sin(t)./t)*0.5;
        
    % store in a array (to plot later)
    XR = [XR xr]; YR = [YR yr];
        
    % compute cross product
    v1 = [cos(theta), sin(theta), 0];
    v2 = [xr, yr, 0] -[x, y, 0]; 
    dv = dot(v2, v1)/(norm(v2));
    if abs(dv-1) < 1e-5; dv = 1; end % accuracy problem in Matlab
   
    phi = acos( dv )*180/pi; % deg. system 
    cp = cross(v1, v2);
    
    % Error in orientation
    phiE = sign(cp(3))*phi;
    
    % Error euclidean
    D = norm( v2 );
   
    % PID for euclidean error
    IE = IE + D*dt;
    DE = (D - PE)/dt;
    v = Kp*D  + Ki*IE + Kd*DE;
    PE = D;
    
    % PID for orientation
    IEo = IEo + phiE*dt;
    DEo = (phiE - PEo)/dt;
    w = Kpo*phiE  + Kio*IEo + Kdo*DEo; %%
    PEo = phiE;

    %robot = rospublisher('/cmd_vel') ;
    %velmsg = rosmessage(robot);
    velmsg.Linear.X= v;   %up 
    velmsg.Angular.Z = w; %u theta
    send(robot, velmsg);
    
    Vh(end+1) = v; Wh(end+1) = w;
    pause(0.1)
    
    
    plot(XR,YR);
    hold on
    plot(X, Y);
    hold on
    drawnow

end % simulation/for

velmsg.Linear.X= 0; 
velmsg.Angular.Z = 0;
send(robot, velmsg);


close all
% plot references vs. robot trajectory
plot(XR,YR);
hold on
plot(X, Y);
hold on
plot(XR(end), YR(end), 'or')
hold on
plot(X(end), Y(end), 'ob')
legend('References', 'Robot')
axis square

% figure(2); plot(Vh)
% figure(3); plot(Wh)



% % clear
% % rosshutdown
