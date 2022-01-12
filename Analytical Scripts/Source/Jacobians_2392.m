function [Jac_h, Jac_k, Jac_a, Jac_talus, T_hk, T_ha,T_h_COM_thigh,T_h_COM_shank,T_h_COM_foot] = Jacobians_2392(rx_h,ry_h,rz_h,COM_h,l_h,rz_k,COM_k, l_k,rz_a,COM_a)
% DH notation rotz,transz, transx, rotx
% trans is a 1x3 vector
% Frame used with:
% - z pointing to the right of the right leg (or to the left of the left leg i.e. laterally)
% - x pointing upwards
% - y pointing back

hip_flexion = makehgtform('zrotate', (rz_h), 'xrotate', pi/2);
hip_adduction = makehgtform('zrotate', (rx_h-pi/2), 'xrotate', -pi/2); %hip adduction
hip_rotation_hk_transf = makehgtform('zrotate', (ry_h-pi/2),'translate', [0 0 -l_h], 'xrotate', pi/2); %hip rotation

Z0 = [0;0;1];
P0 = [0;0;0];

T01 = hip_flexion;
P1 = T01(1:3,4);
Z1 = T01(1:3,3);

T02 = T01*hip_adduction;
P2 = T02(1:3,4);
Z2 = T02(1:3,3);

hip_rotation_trans_COM_h = makehgtform('zrotate', ry_h, 'translate', [-COM_h(3) -COM_h(1) COM_h(2)]);
T0_COM_h = T02*hip_rotation_trans_COM_h;
P_COM_h = T0_COM_h(1:3,4);
Jp1_COM_h = cross(Z0, P_COM_h-P0);
Jp2_COM_h = cross(Z1, P_COM_h-P1);
JpEE_COM_h = cross(Z2, P_COM_h-P2);
Jo1 = Z0;
Jo2 = Z1;
Jo_COM_h = Z2;
Jac_h = [Jp1_COM_h Jp2_COM_h JpEE_COM_h; Jo1 Jo2 Jo_COM_h];
T_h_COM_thigh=T0_COM_h;

T03 = T02*hip_rotation_hk_transf;
P3 = T03(1:3,4);
Z3 = T03(1:3,3);

rotz_k_COM_k = makehgtform('zrotate', (rz_k-(pi/2)), 'translate', [-COM_k(2) COM_k(1) COM_k(3)]);
T0_COM_k = T03*rotz_k_COM_k;
P_COM_k = T0_COM_k(1:3,4);
Jp1_COM_k = cross(Z0, P_COM_k-P0);
Jp2_COM_k = cross(Z1, P_COM_k-P1);
Jp3_COM_k = cross(Z2, P_COM_k-P2);
JpEE_COM_k = cross(Z3, P_COM_k-P3);
Jo3 = Z2;
Jo_COM_k = Z3; 
Jac_k = [Jp1_COM_k Jp2_COM_k Jp3_COM_k JpEE_COM_k; Jo1 Jo2 Jo3 Jo_COM_k];
T_hk = T03;
T_h_COM_shank=T0_COM_k;

rotz_k_trans_ka = makehgtform('zrotate', (rz_k-pi/2),'translate', [l_k,0,0]);
T04 = T03*rotz_k_trans_ka;
P4 = T04(1:3,4);
Z4 = T04(1:3,3);
T_ha = T04;
Jp1_talus =cross(Z0,P4-P0);
Jp2_talus =cross(Z1,P4-P1);
Jp3_talus =cross(Z2,P4-P2);
Jp4_talus =cross(Z3,P4-P3);

rotz_a_COM_a = makehgtform('zrotate', (rz_a),'translate', [-COM_a(2) COM_a(1) COM_a(3)]);

T0_COM_a = T04 * rotz_a_COM_a;
P_COM_a = T0_COM_a(1:3,4);
Jp1_COM_a = cross(Z0, P_COM_a-P0);
Jp2_COM_a = cross(Z1, P_COM_a-P1);
Jp3_COM_a = cross(Z2, P_COM_a-P2);
Jp4_COM_a = cross(Z3, P_COM_a-P3);
JpEE_COM_a = cross(Z4, P_COM_a-P4);
Jo4 = Z3;
Jo_COM_a = Z4;
Jac_a = [Jp1_COM_a Jp2_COM_a Jp3_COM_a Jp4_COM_a JpEE_COM_a; Jo1 Jo2 Jo3 Jo4 Jo_COM_a];
T_h_COM_foot=T0_COM_a;

Jac_talus=[Jp1_talus Jp2_talus Jp3_talus Jp4_talus;Jo1 Jo2 Jo3 Jo4];

% rotz_a_trans_a_toes = makehgtform('zrotate', rz_a,'translate', [l_foot,0,0]);
% T_h_toes = T04 * rotz_a_trans_a_toes;
end