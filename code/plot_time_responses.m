function plot_time_responses(sim_data,p,speed_kmh,profile)
% Plot chassis/stretcher displacement, transmitted force and stretcher
% acceleration (with the 0.2 g safety limit) for one run.
t = sim_data.t;
x = sim_data.x;
xdot = sim_data.xdot;
F_trans = sim_data.F_trans;

figure('Name',sprintf('Responses: %s at %d kmh',profile,speed_kmh));
subplot(3,1,1);
plot(t,x(:,3)); hold on; plot(t,x(:,5));
legend('x_{chassis}','x_{stretcher}'); xlabel('t (s)'); ylabel('m');
title('Vertical displacement'); grid on;

subplot(3,1,2);
plot(t,1000*diff([0; x(:,3)])./(diff([0; t]))); % crude velocity approx
xlabel('t (s)'); ylabel('m/s'); title('Chassis velocity (approx)'); grid on;

subplot(3,1,3);
plot(t, F_trans); xlabel('t (s)'); ylabel('N');
title('Force transmitted to patient (stretcher column)'); grid on;

acc_s = gradient(gradient(x(:,5),t),t);
figure('Name',sprintf('Accel stretcher: %s %d kmh',profile,speed_kmh));
plot(t,acc_s); hold on; yline(0.2*9.81,'r--','0.2g');
xlabel('t (s)'); ylabel('m/s^2');
title('Stretcher vertical acceleration'); grid on;
end
