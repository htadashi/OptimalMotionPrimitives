function plot_pendulum(scenario, q_sol, filename, with_obstacles)

[m1, l1, m2, l2, Iz1, Iz2, r1, r2] = get_params_pendulum(scenario);

% MDH Paramters of the robot arm (theta is the free parameter for a revolute joint)
% Link 1 
a1      = 0;
d1      = 0;
alpha1  = 0;

% Link 2
a2      = l1;
d2      = 0;
alpha2  = 0;

% Additional parameters
g = 9.81; % acceleration due to gravity in m/s^2

% Inertial matrices around center of mass
I1 = Iz1;
I2 = Iz2;

% Inertial matrices around link
I1 =  I1 + m1*r1^2;
I2 =  I2 + m2*r2^2;

% Define robot
robot = rigidBodyTree;
robot.Gravity = [0 -g 0];
robot.DataFormat = "row";

% Body 1
body1 = rigidBody('body1');
body1.Mass = m1;
body1.Inertia = [0 0 I1 0 0 0];
body1.CenterOfMass = [r1 0 0];

jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;
jnt1.JointAxis = [0 0 1];
tform = trvec2tform([0, 0., 0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

addBody(robot,body1,'base')

% Body 2
body2 = rigidBody('body2');
body2.Mass = m2;
body2.Inertia = [0 0 I2 0 0 0];
body2.CenterOfMass = [r2 0 0];

jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0; % User defined
jnt2.JointAxis = [0 0 1];
tform2 = trvec2tform([l1, 0, 0]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

% Body EE
bodyEE = rigidBody('endeffector');
bodyEE.Mass = 0;
bodyEE.Inertia = [0 0 0 0 0 0];
bodyEE.CenterOfMass = [0 0 0];


tform3 = trvec2tform([l2, 0, 0]); % User defined
setFixedTransform(bodyEE.Joint,tform3);
addBody(robot,bodyEE,'body2');

% showdetails(robot);
%% Plotting 
% figure
% show(robot,q_sol(:,1)');
% view(2)
% ax = gca;
% ax.Projection = 'orthographic';
% hold on
% plot(points(:,1),points(:,2),'k')
% axis([-0.1 0.7 -0.3 0.5])

figure;
show(robot);
view(0,90)

h =  findobj('type','figure');
n = length(h);


%% Add an obstacle
if with_obstacles == 1
    hold on;
    %draw_circle(0.6,1.3,0.2);
    %hold on;
    plot(0,1.9,'r*')
    %draw_circle(-1.5,-1,0.3);   
    hold on;
end



hold on

framesPerSecond = 50;
r = rateControl(framesPerSecond);
for i = 1:size(q_sol(1,:),2)
    show(robot,q_sol(:,i)','PreservePlot',false);
    drawnow
    

    % save as a gif
   
    frame = getframe(n);   
    im = frame2im(frame);

    [imind,cm] = rgb2ind(im,256);
    if i == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0);
    end

    figure(n)
    waitfor(r);
    figure(n)
 

end





