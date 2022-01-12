function [T_hk, T_ha, T_h_toes] = Transformations_2392(rx_h,ry_h,rz_h,l_h, rz_k, l_k,rz_a,l_foot)

% DH notation rotz,transz, transx, rotx (input in radians)
% trans is a 1x3 vector

% Frame used with:
% - z pointing to the right of the right leg (or to the left of the left leg i.e. laterally)
% - x pointing upwards
% - y pointing back


hip_flexion = makehgtform('zrotate', (rz_h), 'xrotate', pi/2);
hip_adduction = makehgtform('zrotate', (rx_h-pi/2), 'xrotate', -pi/2); %hip adduction

hip_rotation_hk_transf = makehgtform('zrotate', (ry_h-pi/2),'translate', [0 0 -l_h], 'xrotate', pi/2); %hip rotation

T01 = hip_flexion;
T02 = T01*hip_adduction;
T03 = T02*hip_rotation_hk_transf;
T_hk = T03;

rotz_k_trans_ka = makehgtform('zrotate', (rz_k-pi/2),'translate', [l_k,0,0]);

T04 = T03*rotz_k_trans_ka;
T_ha = T04;

rotz_a_trans_a_toes = makehgtform('zrotate', rz_a,'translate', [l_foot,0,0]);

T_h_toes = T04 * rotz_a_trans_a_toes;
end