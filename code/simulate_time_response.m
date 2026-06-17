function sim_data = simulate_time_response(M,C,K,p,tspan,V,profile,Omega,Fe_amp)
% Integrate the 5-DOF equations of motion in the time domain with ode45.
x0 = zeros(10,1); % [x; xdot] for 5 DOF
tOut = linspace(tspan(1),tspan(2),2000);
M_inv = inv(M);
odefun = @(t,xx) state_rhs(t,xx,M_inv,C,K,p,V,profile,Omega,Fe_amp);
opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.02);
[tt,XX] = ode45(odefun,tOut,x0,opts);
sim_data.t = tt;
sim_data.x = XX(:,1:5);
sim_data.xdot = XX(:,6:10);
xs = sim_data.x(:,5); xc = sim_data.x(:,3);
xsd = sim_data.xdot(:,5); xcd = sim_data.xdot(:,3);
F_trans = p.k_s*(xs - xc) + p.c_s*(xsd - xcd);
sim_data.F_trans = F_trans;
end

function dstate = state_rhs(t,xx,M_inv,C,K,p,V,profile,Omega,Fe_amp)
n = length(xx)/2;
x = xx(1:n); xd = xx(n+1:end);
F = zeros(5,1);
wheelbase = p.Lf + p.Lr;
tau = wheelbase / V; % time delay (approx)
y_front = road_profile(profile,V,t, 'front', p);
y_rear  = road_profile(profile,V,t - tau, 'rear', p);
F(1) = p.ktf * y_front;
F(2) = p.ktr * y_rear;
Fe = Fe_amp * sin(Omega*t);
F(3) = F(3) + Fe;
F(4) = F(4) + Fe * p.L_e;
xdd = M_inv * (F - C*xd - K*x);
dstate = [xd; xdd];
end
