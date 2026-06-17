function [k_s, c_s] = design_isolator(m_s, target_fn, zeta)
% Simple isolator design for single DOF mass m_s
omega_n = 2*pi*target_fn;
k_s = m_s * omega_n^2;
c_s = 2 * m_s * omega_n * zeta;
end
