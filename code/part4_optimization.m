% part4_optimization.m
% Part 4: design and optimize the stretcher isolator (k_s, c_s) so the
% vertical acceleration transmitted to the patient never exceeds 0.2 g.
% A grid search minimizes the worst-case stretcher acceleration across all
% test speeds while limiting the relative stretcher/chassis travel.

clear; clc; close all;
g = 9.81;
p.mc  = 1800;
p.Iy  = 3000;
p.muf = 60;
p.mur = 80;
p.kf  = 30000;
p.kr  = 35000;
p.cf  = 4000;
p.cr  = 4500;
p.ktf = 250000;
p.ktr = 250000;
p.Lf  = 1.3;
p.Lr  = 1.7;
p.ms  = 90;
p.Ls  = 0.5;
p.ks0 = 12000;
p.cs0 = 1200;
p.Le  = 1.2;
profileType = "bump";
useEngine   = true;
Tend        = 12;
tStartHit   = 3.0;
speeds_kmh = [20 40 60 80];

rpm = 1200;
omega = 2*pi*(rpm/60);
Me = 250; Ke = 650e3; Ce = 300;
Xe = 0.4e-3;
mue = estimate_unbalance(Me,Ce,Ke,omega,Xe);
fprintf('Estimated engine unbalance m_u*e = %.6f kg*m\n', mue);

ks_base = p.ks0;
cs_base = p.cs0;
BASE = cell(numel(speeds_kmh),1);
for s = 1:numel(speeds_kmh)
    V_kmh = speeds_kmh(s);
    [tB, outB] = simulate_system(p, ks_base, cs_base, V_kmh, profileType, ...
                                useEngine, omega, mue, Tend, tStartHit);
    BASE{s}.t = tB;
    BASE{s}.aSg = outB.aS/g;
end

ks_grid = linspace(6000, 20000, 32);   % N/m
cs_grid = linspace(400,  2500,  32);   % N*s/m
best.J = inf;
best.ks = ks_base;
best.cs = cs_base;
maxRelDispAllow = 0.08; % m
fprintf('\nRunning grid search optimization across ALL speeds...\n');
for i = 1:numel(ks_grid)
    for j = 1:numel(cs_grid)
        ks = ks_grid(i);
        cs = cs_grid(j);
        peakA_all = zeros(numel(speeds_kmh),1);
        peakRel_all = zeros(numel(speeds_kmh),1);
        for s = 1:numel(speeds_kmh)
            V_kmh = speeds_kmh(s);
            [~, out] = simulate_system(p, ks, cs, V_kmh, profileType, ...
                                       useEngine, omega, mue, Tend, tStartHit);
            aS = out.aS;
            rel = out.xs - out.xb;
            peakA_all(s) = max(abs(aS));
            peakRel_all(s) = max(abs(rel));
        end
        worstPeakA = max(peakA_all);
        worstPeakRel = max(peakRel_all);
        J = worstPeakA + 50*max(0, worstPeakRel - maxRelDispAllow);
        if J < best.J
            best.J = J;
            best.ks = ks;
            best.cs = cs;
            best.worstPeakA = worstPeakA;
            best.worstPeakRel = worstPeakRel;
        end
    end
end
fprintf('BEST (global for all speeds): ks = %.1f N/m, cs = %.1f N*s/m\n', best.ks, best.cs);
fprintf('Worst-case peak |a_s| = %.3f m/s^2  (%.3f g)\n', best.worstPeakA, best.worstPeakA/g);
fprintf('Worst-case peak |xs-xb| = %.3f m\n', best.worstPeakRel);

OPT = cell(numel(speeds_kmh),1);
for s = 1:numel(speeds_kmh)
    V_kmh = speeds_kmh(s);
    [tO, outO] = simulate_system(p, best.ks, best.cs, V_kmh, profileType, ...
                                useEngine, omega, mue, Tend, tStartHit);
    OPT{s}.t = tO;
    OPT{s}.aSg = outO.aS/g;
end

figure; hold on;
for s = 1:numel(speeds_kmh)
    plot(BASE{s}.t, BASE{s}.aSg, 'LineWidth', 1.5);
end
yline(0.2, '--', '0.2g limit', 'LineWidth', 1.2);
yline(-0.2,'--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Stretcher acceleration a_s (g)');
title(sprintf('BEFORE Optimization | BUMP | Engine=%d | ks=%.0f cs=%.0f', useEngine, ks_base, cs_base));
legend("20 km/h","40 km/h","60 km/h","80 km/h",'Location','best');

figure; hold on;
for s = 1:numel(speeds_kmh)
    plot(OPT{s}.t, OPT{s}.aSg, 'LineWidth', 1.5);
end
yline(0.2, '--', '0.2g limit', 'LineWidth', 1.2);
yline(-0.2,'--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Stretcher acceleration a_s (g)');
title(sprintf('AFTER Optimization | BUMP | Engine=%d | ks=%.0f cs=%.0f', useEngine, best.ks, best.cs));
legend("20 km/h","40 km/h","60 km/h","80 km/h",'Location','best');

function mue = estimate_unbalance(M,C,K,omega,Xamp)
% X = F0 / sqrt((K-Mw^2)^2 + (Cw)^2),  F0 = (m_u*e)*w^2
    denom = sqrt((K - M*omega^2)^2 + (C*omega)^2);
    mue = Xamp * denom / (omega^2);
end

function [t, out] = simulate_system(p, ks, cs, V_kmh, profileType, useEngine, omega, mue, Tend, tStartHit)
    V = V_kmh/3.6;
    tau = (p.Lf + p.Lr)/V;   % rear delay
    z0 = zeros(10,1);
    tspan = [0 Tend];
    ode = @(t,z) halfcar_ode(t,z,p,ks,cs,V,profileType,useEngine,omega,mue,tau,tStartHit);
    opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
    [t, z] = ode45(ode, tspan, z0, opts);
    x1 = z(:,1); x2 = z(:,2); xc = z(:,3); th = z(:,4); xs = z(:,5);
    xcd = z(:,8); thd = z(:,9);
    xb  = xc + p.Ls*th;
    % accelerations: evaluate ODE once per point
    aS = zeros(size(t));
    for k = 1:numel(t)
        dz = halfcar_ode(t(k), z(k,:).', p, ks, cs, V, profileType, useEngine, omega, mue, tau, tStartHit);
        aS(k) = dz(10); % xsdd
    end
    out.x1=x1; out.x2=x2; out.xc=xc; out.th=th; out.xs=xs;
    out.xb=xb; out.aS=aS;
end

function dz = halfcar_ode(t,z,p,ks,cs,V,profileType,useEngine,omega,mue,tau,tStartHit)
    x1=z(1); x2=z(2); xc=z(3); th=z(4); xs=z(5);
    x1d=z(6); x2d=z(7); xcd=z(8); thd=z(9); xsd=z(10);
    yf  = road_profile(t, V, profileType, tStartHit);
    yr  = road_profile(t - tau, V, profileType, tStartHit);
    yfd = road_profile_dot(t, V, profileType, tStartHit);
    yrd = road_profile_dot(t - tau, V, profileType, tStartHit);
    xf  = xc - p.Lf*th;    xfd = xcd - p.Lf*thd;
    xr  = xc + p.Lr*th;    xrd = xcd + p.Lr*thd;
    xb  = xc + p.Ls*th;    xbd = xcd + p.Ls*thd;
    Fengine = 0;
    if useEngine
        Fengine = (mue*omega^2) * sin(omega*t);
    end
    Fsf = p.kf*(x1 - xf) + p.cf*(x1d - xfd);
    Fsr = p.kr*(x2 - xr) + p.cr*(x2d - xrd);
    Ftf = p.ktf*(yf - x1) + 0*(yfd - x1d);
    Ftr = p.ktr*(yr - x2) + 0*(yrd - x2d);
    Fs  = ks*(xb - xs) + cs*(xbd - xsd);

    x1dd = (Ftf - Fsf)/p.muf;
    x2dd = (Ftr - Fsr)/p.mur;
    xcdd = ( Fsf + Fsr + Fengine - Fs )/p.mc;
    thdd = ( (-p.Lf)*Fsf + (p.Lr)*Fsr + (p.Ls)*(-Fs) )/p.Iy;
    xsdd = ( Fs )/p.ms;
    dz = [x1d; x2d; xcd; thd; xsd;
          x1dd; x2dd; xcdd; thdd; xsdd];
end

function y = road_profile(t, V, type, tStartHit)
    if t < tStartHit
        y = 0; return;
    end
    x = V*(t - tStartHit);
    switch lower(type)
        case "bump"
            h = 0.1;   L = 0.3;
            if x >= 0 && x <= L
                y = (h/2)*(1 - cos(2*pi*x/L));
            else
                y = 0;
            end
        otherwise
            error('Only "bump" used in this script.');
    end
end

function yd = road_profile_dot(t, V, type, tStartHit)
    if t < tStartHit
        yd = 0; return;
    end
    x = V*(t - tStartHit);
    switch lower(type)
        case "bump"
            h = 0.1;   L = 0.3;
            if x >= 0 && x <= L
                dydx = (h*pi/L)*sin(2*pi*x/L);
                yd = dydx * V;
            else
                yd = 0;
            end
        otherwise
            yd = 0;
    end
end
