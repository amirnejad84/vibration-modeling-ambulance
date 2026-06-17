function [m_t, k_t, c_t] = design_TMD(ms, alpha, omega_target, zeta_t)
% Design a tuned mass damper attached to mass ms
% alpha: mass ratio m_t / ms (e.g., 0.05..0.2)
m_t = alpha * ms;
k_t = m_t * omega_target^2;
c_t = 2 * m_t * omega_target * zeta_t;
end
