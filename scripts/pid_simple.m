niter = 2000;

inertia = 0.002;
length = 0.260*cosd(45);
mass = 0.320;

x = [-pi / 4, 0]; % Current state (position and velocity)
t = 0; % target position

% good values p=2.0, i=0.2 d=1
p_angle = 6; % desired angular speed in rad/s for error 1 rad.

% control output for angular speed error 1 rad/s.
p_rate = 0.1;
i_rate = 0.05;
d_rate = 0.005;

bias = 0; % amount by which the acceleration is off

torq = @(c) (2*2*c*(mass/3)*length)
a = @(c) ( (torq(c) / inertia) + bias); % acceleration as a function of control input


pts = [];

dt = 0.001;
lastE = 0;
sumE = 0;
for it = 1:niter
    
   t = p_angle * (0 - x(1));
    
   % Error and derivative of error
   e = t - x(2);
   de = (e - lastE) + dt;
   
   c = (p_rate * e) + (i_rate * sumE) + (d_rate * de);
   
   
   
   % System dynamics
   ai = a(c);
   x(1) = x(1) + x(2)*dt + 0.5*ai*(dt*dt);
   x(2) = x(2) + dt*ai;
   
   pts = [pts x(1)]; % Record current position
   
   % Update integral term
   sumE = sumE + e*dt;
   lastE = e;
end

figure;
hold on;

ylim([-2, 2])
plot(1:niter, pts, 'b');

plot([0, 1000], [t, t], 'r');

if bias ~= t
    plot([0, 1000], [bias, bias], 'g-');
end

legend('State', 'Target', 'Bias')

hold off;

