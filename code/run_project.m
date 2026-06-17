% run_project.m
% Main driver for the 5-DOF half-car ambulance/stretcher vibration model.
% Builds the M, K, C matrices, runs modal analysis, simulates the time
% response over several road profiles and speeds, and proposes an
% isolator / tuned-mass-damper (TMD) design.

clear; close all; clc;
mode = 'full5DOF';

p = struct();
p.m_c  = 1800;          % chassis mass [kg]
p.Iyc  = 3000;          % chassis pitch inertia about CG [kg*m^2]
p.m_uf = 60;            % front unsprung mass [kg]
p.m_ur = 80;            % rear unsprung mass [kg]
p.kf   = 30000;         % front suspension stiffness [N/m]
p.kr   = 35000;         % rear suspension stiffness [N/m]
p.cf   = 4000;          % front suspension damping [N*s/m]
p.cr   = 4500;          % rear suspension damping [N*s/m]
p.ktf  = 250000;        % front tire stiffness [N/m]
p.ktr  = 250000;        % rear tire stiffness [N/m]
p.Lf   = 1.3;           % front axle to CG [m]
p.Lr   = 1.7;           % rear axle to CG [m]
p.m_s  = 90;            % stretcher + patient mass [kg]
p.Ls   = 0.5;           % stretcher mount to CG [m]
p.k_s  = 12000;         % stretcher isolator stiffness [N/m]
p.c_s  = 1200;          % stretcher isolator damping [N*s/m]
p.L_e  = 1.2;           % engine to CG [m]

p.m_e = 250;            % engine effective mass [kg]
p.e   = 0.004;          % engine eccentricity [m]
p.engine_omega_rpm = 1200;
p.engine_c = 650;
p.engine_k = 3e5;
p.m_override_1dof = 5000;

if strcmp(mode,'full5DOF')
    [M,K,C] = build_MKC_5DOF(p);
    fprintf('Built 5-DOF M,K,C matrices.\n');
else
    M = p.m_override_1dof;
    K = 1e4;
    C = 100;
    fprintf('Built 1-DOF override M,K,C.\n');
end

if strcmp(mode,'full5DOF')
    [Phi,omega_n,frequencies_Hz] = modal_analysis(K,M);
    disp('Natural frequencies (Hz):');
    disp(frequencies_Hz');
    plot_modes(Phi,frequencies_Hz);
end

tspan = [0 12];               % seconds
speeds_kmh = [20 40 60 80];   % km/h to test
profile_list = {'bump','cushion'}; % profiles from project

Omega  = 2*pi*(p.engine_omega_rpm/60);
Fe_amp = p.m_e * p.e * (Omega^2); % amplitude for engine vertical forcing (approx)

for pi=1:numel(profile_list)
    profile = profile_list{pi};
    for s = speeds_kmh
        V = s/3.6; % m/s
        fprintf('\nSimulating profile=%s, speed=%d km/h ...\n',profile,s);
        sim_data = simulate_time_response(M,C,K,p,tspan,V,profile,Omega,Fe_amp);
        plot_time_responses(sim_data,p,s,profile);
    end
end

% Example isolator design (single DOF target)
target_fn = 1.0;
zeta = 0.2;
[k_s_design, c_s_design] = design_isolator(p.m_s,target_fn,zeta);
fprintf('\nExample isolator design for stretcher: k_s = %.1f N/m, c_s = %.1f N*s/m\n',k_s_design,c_s_design);

% Example tuned-mass-damper tuned to first natural frequency
alpha = 0.1;
[m_t, k_t, c_t] = design_TMD(p.m_s,alpha,2*pi*frequencies_Hz(1),0.08);
fprintf('Suggested TMD: m_t=%.2f kg, k_t=%.1f N/m, c_t=%.2f N*s/m\n',m_t,k_t,c_t);

fprintf('\nDone. Inspect figures and adjust parameters as desired.\n');
