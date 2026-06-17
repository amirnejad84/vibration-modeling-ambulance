function [Phi,omega_n,frequencies_Hz] = modal_analysis(K,M)
% Solve the generalized eigenvalue problem K*phi = omega^2 * M*phi
% and return mode shapes and natural frequencies sorted ascending.
[Phi,D] = eig(K,M);
omega2 = diag(D);
omega_n = sqrt(abs(omega2));
[omega_n, idx] = sort(omega_n);
Phi = Phi(:,idx);
frequencies_Hz = omega_n/(2*pi);
end
