function [M,K,C] = build_MKC_5DOF(p)
% Assemble the mass, stiffness and damping matrices of the 5-DOF half-car
% model. DOF order: [x_f, x_r, x_c, theta, x_s].
muf = p.m_uf; mur = p.m_ur; mc = p.m_c; Iyc = p.Iyc; ms = p.m_s;
kf = p.kf; kr = p.kr; ktf = p.ktf; ktr = p.ktr; ks = p.k_s;
cf = p.cf; cr = p.cr; cs = p.c_s;
Lf = p.Lf; Lr = p.Lr;

M = diag([muf, mur, mc, Iyc, ms]);

K = zeros(5);
i1=1; i2=2; i3=3; i4=4; i5=5;
K(i1,i1) = ktf + kf;
K(i1,i3) = -kf;
K(i1,i4) = -kf*Lf;
K(i2,i2) = ktr + kr;
K(i2,i3) = -kr;
K(i2,i4) = kr*Lr;
K(i3,i1) = -kf;
K(i3,i2) = -kr;
K(i3,i3) = kf + kr + ks;
K(i3,i4) = -kf*Lf + kr*Lr;
K(i3,i5) = -ks;
K(i4,i1) = -kf*Lf;
K(i4,i2) = kr*Lr;
K(i4,i3) = -kf*Lf + kr*Lr;
K(i4,i4) = kf*Lf^2 + kr*Lr^2;
K(i5,i3) = -ks;
K(i5,i5) = ks;
K = (K + K.')/2;

C = zeros(5);
C(i1,i1) = cf;      C(i1,i3) = -cf;     C(i1,i4) = -cf*Lf;
C(i3,i1) = -cf;     C(i3,i3) = cf;
C(i4,i1) = -cf*Lf;  C(i4,i3) = -cf*Lf;  C(i4,i4) = cf*Lf^2;
C(i2,i2) = C(i2,i2) + cr;     C(i2,i3) = -cr;      C(i2,i4) = cr*Lr;
C(i3,i2) = C(i3,i2) - cr;     C(i3,i3) = C(i3,i3) + cr;
C(i4,i2) = C(i4,i2) + cr*Lr;  C(i4,i3) = C(i4,i3) + cr*Lr;  C(i4,i4) = C(i4,i4) + cr*Lr^2;
C(i3,i3) = C(i3,i3) + cs;
C(i3,i5) = C(i3,i5) - cs;
C(i5,i3) = C(i5,i3) - cs;
C(i5,i5) = C(i5,i5) + cs;
C = (C + C.')/2;
end
