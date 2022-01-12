function [Jacs, Transformations] = getH3Jacobians(model,Q_act,Inertias,Lengths,grav_comp_flag,k)
%[Jac_h, Jac_k, Jac_a, Jac_talus, T_hk, T_ha,T_h_COM_thigh,T_h_COM_shank,T_h_COM_foot,T_hip_to_toes]=Jacs_2392(rx_h,ry_h,rz_h,COM_h,l_h,rz_k,COM_k, l_k,rz_a,COM_a)
% DH notation rotz,transz, transx, rotx
% trans is a 1x3 vector
% BaseFrame used with:
% - z pointing to the right (can we change this to anatomical planes????)
% - x pointing upwards
% - y pointing back
% designed to be used with the H3 2392 model, 
% q_act = live limb position of the exoskeleton

%% Right Exo Leg
hip_flexion = makehgtform('zrotate', (Q_act.(['H3_hip_flexion_r'])(k)), 'xrotate', pi/2); %creates a 4x4 transform matrix which has rotational and translational commands Q is theta of the joints, act is the actial ref is the desired
hip_adduction = makehgtform('zrotate', (Q_act.(['H3_hip_adduction_r'])(k)-pi/2), 'xrotate', -pi/2); %hip adduction
hip_rotation_hk_transf = makehgtform('zrotate', (Q_act.(['H3_hip_rotation_r'])(k)-pi/2),'translate', [0 0 -Lengths.exo.thigh_r], 'xrotate', pi/2); %hip rotation


T01 = hip_flexion; % x pointing upward, y to the right, z forwards 
T02 = T01*hip_adduction;% x left, y back, z up combine the adduction and flexion matrix for the hip



hip_motor_trans_thigh = osimVec3ToArray(model.getJointSet().get('hip_motor_r_H3_thigh_r').get_frames(0).get_translation()); %accesses the opensim thigh translational data and creates a 1x3 matlab matrix is this the gravity centrre of the joint?
hip_rotation_trans_COM_h = makehgtform('zrotate', Q_act.(['H3_hip_rotation_r'])(k), 'translate', [-Inertias.exo.COMs.(['H3_thigh_r'])(3)-hip_motor_trans_thigh(3) -Inertias.exo.COMs.(['H3_thigh_r'])(1)-hip_motor_trans_thigh(1) Inertias.exo.COMs.(['H3_thigh_r'])(2)+hip_motor_trans_thigh(2)]); %accesses the gravity centres of the joint
T0_COM_thigh = T02*hip_rotation_trans_COM_h; %combine rotation, flexion and adduction matrix for the hgip
Transformations.exo.right.hip_to_COM_thigh=T0_COM_thigh; %save as hip to centre of mass data

hip_rotation_trans_COM_hip_motor = makehgtform('zrotate', Q_act.(['H3_hip_rotation_r'])(k), 'translate', [-Inertias.exo.COMs.(['hip_motor_r'])(3) -Inertias.exo.COMs.(['hip_motor_r'])(1) Inertias.exo.COMs.(['hip_motor_r'])(2)]);% centre of mass of hip
T0_COM_hip_motor = T02*hip_rotation_trans_COM_hip_motor;  %hip matrix with rotation, adduction and flexion
Transformations.exo.right.hip_to_COM_hip_motor=T0_COM_hip_motor; %save the centre of mass data


T03 = T02*hip_rotation_hk_transf; %x pointing to the front, y pointing upwards, z pointing to the right
Transformations.exo.right.hip_to_knee = T03;

knee_motor_trans_shank = osimVec3ToArray(model.getJointSet().get('knee_motor_r_H3_shank_r').get_frames(0).get_translation());
rotz_k_COM_shank = makehgtform('zrotate', (Q_act.(['H3_knee_angle_r'])(k)-(pi/2)), 'translate', [-Inertias.exo.COMs.(['H3_shank_r'])(2)-knee_motor_trans_shank(2) Inertias.exo.COMs.(['H3_shank_r'])(1)+knee_motor_trans_shank(1) Inertias.exo.COMs.(['H3_shank_r'])(3)+knee_motor_trans_shank(3)]); %x pointing down, y pointing forwards, z pointing to the right
T0_COM_shank = T03*rotz_k_COM_shank; 
Transformations.exo.right.hip_to_COM_shank=T0_COM_shank;

rotz_k_trans_ka = makehgtform('zrotate', (Q_act.(['H3_knee_angle_r'])(k)-pi/2),'translate', [Lengths.exo.shank_r,0,0]);
T04 = T03*rotz_k_trans_ka; % x pointing down, y pointing forwards, z pointing to the right
Transformations.exo.right.hip_to_ankle = T04;

rotz_a_toes = makehgtform('zrotate', Q_act.(['H3_ankle_angle_r'])(k),'translate', [-Inertias.exo.COMs.(['H3_foot_r'])(2) Lengths.exo.foot_r Inertias.exo.COMs.(['H3_foot_r'])(3)]);
T0_toes = T04 * rotz_a_toes;
Transformations.exo.right.hip_to_toes = T0_toes;

ankle_motor_trans_foot = osimVec3ToArray(model.getJointSet().get('ankle_motor_r_H3_foot_r').get_frames(0).get_translation());
rotz_a_COM_a = makehgtform('zrotate', Q_act.(['H3_ankle_angle_r'])(k),'translate', [-Inertias.exo.COMs.(['H3_foot_r'])(2)-ankle_motor_trans_foot(2) Inertias.exo.COMs.(['H3_foot_r'])(1)+ankle_motor_trans_foot(1) Inertias.exo.COMs.(['H3_foot_r'])(3)+ankle_motor_trans_foot(3)]);
T0_COM_a = T04 * rotz_a_COM_a;
Transformations.exo.right.hip_to_COM_foot=T0_COM_a;

hip_motor_sup_thigh_cuff_offset = osimVec3ToArray(model.getJointSet().get('hip_motor_r_H3_sup_thigh_girth_r').get_frames(0).get_translation());
hip_motor_COM_sup_thigh_cuff_offset = hip_motor_sup_thigh_cuff_offset + Inertias.exo.COMs.H3_sup_thigh_girth_r;
hip_rotation_trans_COM_sup_thigh_cuff = makehgtform('zrotate', Q_act.(['H3_hip_rotation_r'])(k), 'translate', [-hip_motor_COM_sup_thigh_cuff_offset(3) -hip_motor_COM_sup_thigh_cuff_offset(1) hip_motor_COM_sup_thigh_cuff_offset(2)]);
T0_COM_sup_thigh_cuff = T02 * hip_rotation_trans_COM_sup_thigh_cuff;
Transformations.exo.right.hip_to_COM_sup_thigh_cuff=T0_COM_sup_thigh_cuff; %gravity comp for exo on the human, thigh

knee_shell_inf_thigh_cuff_offset = osimVec3ToArray(model.getJointSet().get('H3_knee_shell_r_H3_inf_thigh_girth_r').get_frames(0).get_translation());
knee_shell_COM_inf_thigh_cuff_offset = knee_shell_inf_thigh_cuff_offset + Inertias.exo.COMs.H3_inf_thigh_girth_r;
knee_trans_COM_inf_thigh_cuff = makehgtform('translate', [knee_shell_COM_inf_thigh_cuff_offset(1), knee_shell_COM_inf_thigh_cuff_offset(2) knee_shell_COM_inf_thigh_cuff_offset(3)]);
T0_COM_inf_thigh_cuff = T03 * knee_trans_COM_inf_thigh_cuff;
Transformations.exo.right.hip_to_COM_inf_thigh_cuff=T0_COM_inf_thigh_cuff;

knee_trans_COM_knee_motor = makehgtform('zrotate', (Q_act.(['H3_knee_angle_r'])(k)-pi/2),'translate', [-Inertias.exo.COMs.knee_motor_r(2), Inertias.exo.COMs.knee_motor_r(1) Inertias.exo.COMs.knee_motor_r(3)]);
T0_COM_knee_motor = T03 * knee_trans_COM_knee_motor;
Transformations.exo.right.hip_to_COM_knee_motor=T0_COM_knee_motor;

knee_trans_COM_knee_shell = makehgtform('translate', [Inertias.exo.COMs.H3_knee_shell_r(1), Inertias.exo.COMs.H3_knee_shell_r(2) Inertias.exo.COMs.H3_knee_shell_r(3)]);
T0_COM_knee_shell = T03 * knee_trans_COM_knee_shell;
Transformations.exo.right.hip_to_COM_knee_shell=T0_COM_knee_shell;

knee_motor_sup_shank_cuff_offset = osimVec3ToArray(model.getJointSet().get('knee_motor_r_H3_sup_shank_girth_r').get_frames(0).get_translation());
knee_motor_COM_sup_shank_cuff_offset = knee_motor_sup_shank_cuff_offset + Inertias.exo.COMs.H3_sup_shank_girth_r;
knee_rotation_trans_COM_sup_shank_cuff = makehgtform('zrotate', Q_act.(['H3_knee_angle_r'])(k), 'translate', [knee_motor_COM_sup_shank_cuff_offset(1), knee_motor_COM_sup_shank_cuff_offset(2) knee_motor_COM_sup_shank_cuff_offset(3)]);
T0_COM_sup_shank_cuff = T03 * knee_rotation_trans_COM_sup_shank_cuff;
Transformations.exo.right.hip_to_COM_sup_shank_cuff=T0_COM_sup_shank_cuff;

ankle_shell_inf_shank_cuff_offest = osimVec3ToArray(model.getJointSet().get('H3_ankle_shell_r_H3_inf_shank_girth_r').get_frames(0).get_translation());
ankle_shell_COM_inf_shank_cuff_offset = ankle_shell_inf_shank_cuff_offest + Inertias.exo.COMs.H3_inf_shank_girth_r;
ankle_trans_COM_inf_shank_cuff = makehgtform('translate', [-ankle_shell_COM_inf_shank_cuff_offset(2), ankle_shell_COM_inf_shank_cuff_offset(1) ankle_shell_COM_inf_shank_cuff_offset(3)]);
T0_COM_inf_shank_cuff = T04 * ankle_trans_COM_inf_shank_cuff;
Transformations.exo.right.hip_to_COM_inf_shank_cuff=T0_COM_inf_shank_cuff;

ankle_trans_COM_ankle_motor = makehgtform('zrotate', Q_act.(['H3_ankle_angle_r'])(k),'translate', [-Inertias.exo.COMs.ankle_motor_r(2), Inertias.exo.COMs.ankle_motor_r(1) Inertias.exo.COMs.ankle_motor_r(3)]);
T0_COM_ankle_motor = T04 * ankle_trans_COM_ankle_motor;
Transformations.exo.right.hip_to_COM_knee_shell=T0_COM_ankle_motor;

ankle_trans_COM_ankle_shell = makehgtform('translate', [-Inertias.exo.COMs.H3_ankle_shell_r(2), Inertias.exo.COMs.H3_ankle_shell_r(1) Inertias.exo.COMs.H3_ankle_shell_r(3)]);
T0_COM_ankle_shell = T04 * ankle_trans_COM_ankle_shell;
Transformations.exo.right.hip_to_COM_knee_shell=T0_COM_ankle_shell;

if grav_comp_flag
    
    Z0 = [0;0;1];
    P0 = [0;0;0];
    
    Z1 = T01(1:3,3); % isolate rotational column z axis, hip flexion
    P1 = T01(1:3,4); % isolate translational column
    
    Z2 = T02(1:3,3); % isolate the rotational z coordinate for the hip abduction and aduction
    P2 = T02(1:3,4); % isolate the translational column for the hip abduction and aduction
    
    Jo1 = Z0; %Jo is orientation jacobian, jp is the positional jacobian 
    Jo2 = Z1;
    Jo3 = Z2;

    P_COM_hip_motor = T0_COM_hip_motor(1:3,4); %isolate the translational vector in the abduction, flexion and rotational transform
    Jp1_COM_hip_motor = cross(Z0, P_COM_hip_motor-P0); %calculate the cross product berween rotational z transform and
    Jp2_COM_hip_motor = cross(Z1, P_COM_hip_motor-P1); %where does P_COM_hi_motor come from???????????? why does one subtract the translational column?
    JpEE_COM_hip_motor = cross(Z2, P_COM_hip_motor-P2); % what's this subtraction doing? Is it calculating the gravity for just the hip, line before hip and shank and line before that the hip shank and foot?
    Jacs.exo.right.hip_to_COM_hip_motor = [Jp1_COM_hip_motor Jp2_COM_hip_motor JpEE_COM_hip_motor;...
                                            Jo1 Jo2 Jo3];
    
    P_COM_thigh = T0_COM_thigh(1:3,4);
    Jp1_COM_thigh = cross(Z0, P_COM_thigh-P0);
    Jp2_COM_thigh = cross(Z1, P_COM_thigh-P1);
    JpEE_COM_thigh = cross(Z2, P_COM_thigh-P2);
    Jacs.exo.right.hip_to_COM_thigh = [Jp1_COM_thigh Jp2_COM_thigh JpEE_COM_thigh;...   
                                            Jo1 Jo2 Jo3];
    
    P_COM_sup_thigh_cuff = T0_COM_sup_thigh_cuff(1:3,4);
    Jp1_COM_sup_thigh_cuff = cross(Z0, P_COM_sup_thigh_cuff-P0);
    Jp2_COM_sup_thigh_cuff = cross(Z1, P_COM_sup_thigh_cuff-P1);
    JpEE_COM_sup_thigh_cuff = cross(Z2, P_COM_sup_thigh_cuff-P2);
    Jacs.exo.right.hip_to_COM_sup_thigh_cuff = [Jp1_COM_sup_thigh_cuff Jp2_COM_sup_thigh_cuff JpEE_COM_sup_thigh_cuff;...
                                            Jo1 Jo2 Jo3];

    P_COM_inf_thigh_cuff = T0_COM_inf_thigh_cuff(1:3,4);
    Jp1_COM_inf_thigh_cuff = cross(Z0, P_COM_inf_thigh_cuff-P0);
    Jp2_COM_inf_thigh_cuff = cross(Z1, P_COM_inf_thigh_cuff-P1);
    JpEE_COM_inf_thigh_cuff = cross(Z2, P_COM_inf_thigh_cuff-P2);
    Jacs.exo.right.hip_to_COM_inf_thigh_cuff = [Jp1_COM_inf_thigh_cuff Jp2_COM_inf_thigh_cuff JpEE_COM_inf_thigh_cuff; ...
                                            Jo1 Jo2 Jo3];
    
    
    P_COM_knee_shell = T0_COM_knee_shell(1:3,4);
    Jp1_COM_knee_shell = cross(Z0, P_COM_knee_shell-P0);
    Jp2_COM_knee_shell = cross(Z1, P_COM_knee_shell-P1);
    JpEE_COM_knee_shell = cross(Z2, P_COM_knee_shell-P2);
    Jacs.exo.right.hip_to_COM_knee_shell = [Jp1_COM_knee_shell Jp2_COM_knee_shell JpEE_COM_knee_shell;...
                                            Jo1 Jo2 Jo3];

    P3 = T03(1:3,4);
    Z3 = T03(1:3,3);
    Jo4 = Z3;
    
    P_COM_knee_motor = T0_COM_knee_motor(1:3,4);
    Jp1_knee_motor =cross(Z0,P_COM_knee_motor-P0);
    Jp2_knee_motor =cross(Z1,P_COM_knee_motor-P1);
    Jp3_knee_motor =cross(Z2,P_COM_knee_motor-P2);
    Jp4_knee_motor =cross(Z3,P_COM_knee_motor-P3);
    Jacs.exo.right.hip_to_COM_knee_motor = [Jp1_knee_motor Jp2_knee_motor Jp3_knee_motor Jp4_knee_motor; ...
                                        Jo1 Jo2 Jo3 Jo4];
    
    P_COM_shank = T0_COM_shank(1:3,4);
    Jp1_COM_shank = cross(Z0, P_COM_shank-P0);
    Jp2_COM_shank = cross(Z1, P_COM_shank-P1);
    Jp3_COM_shank = cross(Z2, P_COM_shank-P2);
    JpEE_COM_shank = cross(Z3, P_COM_shank-P3);
    Jacs.exo.right.hip_to_COM_shank = [Jp1_COM_shank Jp2_COM_shank Jp3_COM_shank JpEE_COM_shank; ...
                                        Jo1 Jo2 Jo3 Jo4];
    
    P_COM_sup_shank_cuff = T0_COM_sup_shank_cuff(1:3,4);
    Jp1_COM_sup_shank_cuff = cross(Z0, P_COM_sup_shank_cuff-P0);
    Jp2_COM_sup_shank_cuff = cross(Z1, P_COM_sup_shank_cuff-P1);
    Jp3_COM_sup_shank_cuff = cross(Z2, P_COM_sup_shank_cuff-P2);
    JpEE_COM_sup_shank_cuff = cross(Z3, P_COM_sup_shank_cuff-P3);
    Jacs.exo.right.hip_to_COM_sup_shank_cuff = [Jp1_COM_sup_shank_cuff Jp2_COM_sup_shank_cuff Jp3_COM_sup_shank_cuff JpEE_COM_sup_shank_cuff; ...
                                                Jo1 Jo2 Jo3 Jo4];

    P_COM_inf_shank_cuff = T0_COM_inf_shank_cuff(1:3,4);
    Jp1_COM_inf_shank_cuff = cross(Z0, P_COM_inf_shank_cuff-P0);
    Jp2_COM_inf_shank_cuff = cross(Z1, P_COM_inf_shank_cuff-P1);
    Jp3_COM_inf_shank_cuff = cross(Z2, P_COM_inf_shank_cuff-P2);
    JpEE_COM_inf_shank_cuff = cross(Z3, P_COM_inf_shank_cuff-P3);
    Jacs.exo.right.hip_to_COM_inf_shank_cuff = [Jp1_COM_inf_shank_cuff Jp2_COM_inf_shank_cuff Jp3_COM_inf_shank_cuff JpEE_COM_inf_shank_cuff; ...
                                                Jo1 Jo2 Jo3 Jo4];

    P_COM_ankle_shell = T0_COM_ankle_shell(1:3,4);
    Jp1_COM_ankle_shell =cross(Z0,P_COM_ankle_shell-P0);
    Jp2_COM_ankle_shell =cross(Z1,P_COM_ankle_shell-P1);
    Jp3_COM_ankle_shell =cross(Z2,P_COM_ankle_shell-P2);
    Jp4_COM_ankle_shell =cross(Z3,P_COM_ankle_shell-P3);
    Jacs.exo.right.hip_to_COM_ankle_shell=[Jp1_COM_ankle_shell Jp2_COM_ankle_shell Jp3_COM_ankle_shell Jp4_COM_ankle_shell;...
                                           Jo1 Jo2 Jo3 Jo4];

    P4 = T04(1:3,4);
    Z4 = T04(1:3,3);
    Jo5 = Z4;
    
    P_COM_ankle_motor = T0_COM_ankle_motor(1:3,4);
    Jp1_COM_ankle_motor =cross(Z0,P_COM_ankle_motor-P0);
    Jp2_COM_ankle_motor =cross(Z1,P_COM_ankle_motor-P1);
    Jp3_COM_ankle_motor =cross(Z2,P_COM_ankle_motor-P2);
    Jp4_COM_ankle_motor =cross(Z3,P_COM_ankle_motor-P3);
    Jp5_COM_ankle_motor =cross(Z4,P_COM_ankle_motor-P4);
    Jacs.exo.right.hip_to_COM_ankle_motor=[Jp1_COM_ankle_motor Jp2_COM_ankle_motor Jp3_COM_ankle_motor Jp4_COM_ankle_motor Jp5_COM_ankle_motor;...
                                    Jo1 Jo2 Jo3 Jo4 Jo5];

    
    P_COM_a = T0_COM_a(1:3,4);
    Jp1_COM_a = cross(Z0, P_COM_a-P0);
    Jp2_COM_a = cross(Z1, P_COM_a-P1);
    Jp3_COM_a = cross(Z2, P_COM_a-P2);
    Jp4_COM_a = cross(Z3, P_COM_a-P3);
    JpEE_COM_a = cross(Z4, P_COM_a-P4);
    Jacs.exo.right.hip_to_COM_foot = [Jp1_COM_a Jp2_COM_a Jp3_COM_a Jp4_COM_a JpEE_COM_a; Jo1 Jo2 Jo3 Jo4 Jo5];
else
    Jacs=[];
end

% Jacs.right.hip_to_ankle=[Jp1_talus Jp2_talus Jp3_talus Jp4_talus;Jo1 Jo2 Jo3 Jo4];


%% Left Exo Leg
hip_flexion = makehgtform('zrotate', (Q_act.(['H3_hip_flexion_l'])(k)), 'xrotate', pi/2);
hip_adduction = makehgtform('zrotate', (Q_act.(['H3_hip_adduction_l'])(k)-pi/2), 'xrotate', -pi/2); %hip adduction
hip_rotation_hk_transf = makehgtform('zrotate', (Q_act.(['H3_hip_rotation_l'])(k)-pi/2),'translate', [0 0 -Lengths.exo.thigh_l], 'xrotate', pi/2); %hip rotation


T01 = hip_flexion; % x pointing upward, y to the right, z forwards
T02 = T01*hip_adduction;% x left, y back, z up

hip_motor_trans_thigh = osimVec3ToArray(model.getJointSet().get('hip_motor_l_H3_thigh_l').get_frames(0).get_translation());
hip_rotation_trans_COM_h = makehgtform('zrotate', Q_act.(['H3_hip_rotation_l'])(k), 'translate', [-Inertias.exo.COMs.(['H3_thigh_l'])(3)-hip_motor_trans_thigh(3) -Inertias.exo.COMs.(['H3_thigh_l'])(1)-hip_motor_trans_thigh(1) Inertias.exo.COMs.(['H3_thigh_l'])(2)+hip_motor_trans_thigh(2)]);
T0_COM_thigh = T02*hip_rotation_trans_COM_h;
Transformations.exo.left.hip_to_COM_thigh=T0_COM_thigh;

hip_rotation_trans_COM_hip_motor = makehgtform('zrotate', Q_act.(['H3_hip_rotation_l'])(k), 'translate', [-Inertias.exo.COMs.(['hip_motor_l'])(3) -Inertias.exo.COMs.('hip_motor_l')(1) Inertias.exo.COMs.('hip_motor_l')(2)]);
T0_COM_hip_motor = T02*hip_rotation_trans_COM_hip_motor;
Transformations.exo.left.hip_to_COM_hip_motor=T0_COM_hip_motor;


T03 = T02*hip_rotation_hk_transf; %x pointing to the front, y pointing upwards, z pointing to the right
Transformations.exo.left.hip_to_knee = T03;

knee_motor_trans_shank = osimVec3ToArray(model.getJointSet().get('knee_motor_l_H3_shank_l').get_frames(0).get_translation());
rotz_k_COM_shank = makehgtform('zrotate', (Q_act.(['H3_knee_angle_l'])(k)-(pi/2)), 'translate', [-Inertias.exo.COMs.(['H3_shank_l'])(2)-knee_motor_trans_shank(2) Inertias.exo.COMs.(['H3_shank_l'])(1)+knee_motor_trans_shank(1) Inertias.exo.COMs.(['H3_shank_l'])(3)+knee_motor_trans_shank(3)]); %x pointing down, y pointing forwards, z pointing to the right
T0_COM_shank = T03*rotz_k_COM_shank; 
Transformations.exo.left.hip_to_COM_shank=T0_COM_shank;

rotz_k_trans_ka = makehgtform('zrotate', (Q_act.(['H3_knee_angle_l'])(k)-pi/2),'translate', [Lengths.exo.shank_l,0,0]);
T04 = T03*rotz_k_trans_ka; % x pointing down, y pointing forwards, z pointing to the right
Transformations.exo.left.hip_to_ankle = T04;

rotz_a_toes = makehgtform('zrotate', Q_act.(['H3_ankle_angle_l'])(k),'translate', [-Inertias.exo.COMs.(['H3_foot_l'])(2) Lengths.exo.foot_l Inertias.exo.COMs.(['H3_foot_l'])(3)]);
T0_toes = T04 * rotz_a_toes;
Transformations.exo.left.hip_to_toes = T0_toes;

ankle_motor_trans_foot = osimVec3ToArray(model.getJointSet().get('ankle_motor_l_H3_foot_l').get_frames(0).get_translation());
rotz_a_COM_a = makehgtform('zrotate', Q_act.(['H3_ankle_angle_l'])(k),'translate', [-Inertias.exo.COMs.(['H3_foot_l'])(2)-ankle_motor_trans_foot(2) Inertias.exo.COMs.(['H3_foot_l'])(1)+ankle_motor_trans_foot(1) Inertias.exo.COMs.(['H3_foot_l'])(3)+ankle_motor_trans_foot(3)]);
T0_COM_a = T04 * rotz_a_COM_a;
Transformations.exo.left.hip_to_COM_foot=T0_COM_a;

hip_motor_sup_thigh_cuff_offset = osimVec3ToArray(model.getJointSet().get('hip_motor_l_H3_sup_thigh_girth_l').get_frames(0).get_translation());
hip_motor_COM_sup_thigh_cuff_offset = hip_motor_sup_thigh_cuff_offset + Inertias.exo.COMs.H3_sup_thigh_girth_l;
hip_rotation_trans_COM_sup_thigh_cuff = makehgtform('zrotate', Q_act.(['H3_hip_rotation_l'])(k), 'translate', [-hip_motor_COM_sup_thigh_cuff_offset(3) -hip_motor_COM_sup_thigh_cuff_offset(1) hip_motor_COM_sup_thigh_cuff_offset(2)]);
T0_COM_sup_thigh_cuff = T02 * hip_rotation_trans_COM_sup_thigh_cuff;
Transformations.exo.left.hip_to_COM_sup_thigh_cuff=T0_COM_sup_thigh_cuff;

knee_shell_inf_thigh_cuff_offset = osimVec3ToArray(model.getJointSet().get('H3_knee_shell_l_H3_inf_thigh_girth_l').get_frames(0).get_translation());
knee_shell_COM_inf_thigh_cuff_offset = knee_shell_inf_thigh_cuff_offset + Inertias.exo.COMs.H3_inf_thigh_girth_l;
knee_trans_COM_inf_thigh_cuff = makehgtform('translate', [knee_shell_COM_inf_thigh_cuff_offset(1), knee_shell_COM_inf_thigh_cuff_offset(2) knee_shell_COM_inf_thigh_cuff_offset(3)]);
T0_COM_inf_thigh_cuff = T03 * knee_trans_COM_inf_thigh_cuff;
Transformations.exo.left.hip_to_COM_inf_thigh_cuff=T0_COM_inf_thigh_cuff;

knee_trans_COM_knee_motor = makehgtform('zrotate', (Q_act.(['H3_knee_angle_l'])(k)-pi/2),'translate', [-Inertias.exo.COMs.knee_motor_l(2), Inertias.exo.COMs.knee_motor_l(1) Inertias.exo.COMs.knee_motor_l(3)]);
T0_COM_knee_motor = T03 * knee_trans_COM_knee_motor;
Transformations.exo.left.hip_to_COM_knee_motor=T0_COM_knee_motor;

knee_trans_COM_knee_shell = makehgtform('translate', [Inertias.exo.COMs.H3_knee_shell_l(1), Inertias.exo.COMs.H3_knee_shell_l(2) Inertias.exo.COMs.H3_knee_shell_l(3)]);
T0_COM_knee_shell = T03 * knee_trans_COM_knee_shell;
Transformations.exo.left.hip_to_COM_knee_shell=T0_COM_knee_shell;

knee_motor_sup_shank_cuff_offset = osimVec3ToArray(model.getJointSet().get('knee_motor_l_H3_sup_shank_girth_l').get_frames(0).get_translation());
knee_motor_COM_sup_shank_cuff_offset = knee_motor_sup_shank_cuff_offset + Inertias.exo.COMs.H3_sup_shank_girth_l;
knee_rotation_trans_COM_sup_shank_cuff = makehgtform('zrotate', Q_act.(['H3_knee_angle_l'])(k), 'translate', [knee_motor_COM_sup_shank_cuff_offset(1), knee_motor_COM_sup_shank_cuff_offset(2) knee_motor_COM_sup_shank_cuff_offset(3)]);
T0_COM_sup_shank_cuff = T03 * knee_rotation_trans_COM_sup_shank_cuff;
Transformations.exo.left.hip_to_COM_sup_shank_cuff=T0_COM_sup_shank_cuff;

ankle_shell_inf_shank_cuff_offest = osimVec3ToArray(model.getJointSet().get('H3_ankle_shell_l_H3_inf_shank_girth_l').get_frames(0).get_translation());
ankle_shell_COM_inf_shank_cuff_offset = ankle_shell_inf_shank_cuff_offest + Inertias.exo.COMs.H3_inf_shank_girth_l;
ankle_trans_COM_inf_shank_cuff = makehgtform('translate', [-ankle_shell_COM_inf_shank_cuff_offset(2), ankle_shell_COM_inf_shank_cuff_offset(1) ankle_shell_COM_inf_shank_cuff_offset(3)]);
T0_COM_inf_shank_cuff = T04 * ankle_trans_COM_inf_shank_cuff;
Transformations.exo.left.hip_to_COM_inf_shank_cuff=T0_COM_inf_shank_cuff;

ankle_trans_COM_ankle_motor = makehgtform('zrotate', Q_act.(['H3_ankle_angle_l'])(k),'translate', [-Inertias.exo.COMs.ankle_motor_l(2), Inertias.exo.COMs.ankle_motor_l(1) Inertias.exo.COMs.ankle_motor_l(3)]);
T0_COM_ankle_motor = T04 * ankle_trans_COM_ankle_motor;
Transformations.exo.left.hip_to_COM_knee_shell=T0_COM_ankle_motor;

ankle_trans_COM_ankle_shell = makehgtform('translate', [-Inertias.exo.COMs.H3_ankle_shell_l(2), Inertias.exo.COMs.H3_ankle_shell_l(1) Inertias.exo.COMs.H3_ankle_shell_l(3)]);
T0_COM_ankle_shell = T04 * ankle_trans_COM_ankle_shell;
Transformations.exo.left.hip_to_COM_knee_shell=T0_COM_ankle_shell;

if grav_comp_flag
    
    Z0 = [0;0;1];
    P0 = [0;0;0];
    
    Z1 = T01(1:3,3);
    P1 = T01(1:3,4);
    
    Z2 = T02(1:3,3);
    P2 = T02(1:3,4);
    
    Jo1 = Z0;
    Jo2 = Z1;
    Jo3 = Z2;

    P_COM_hip_motor = T0_COM_hip_motor(1:3,4);
    Jp1_COM_hip_motor = cross(Z0, P_COM_hip_motor-P0);
    Jp2_COM_hip_motor = cross(Z1, P_COM_hip_motor-P1);
    JpEE_COM_hip_motor = cross(Z2, P_COM_hip_motor-P2);
    Jacs.exo.left.hip_to_COM_hip_motor = [Jp1_COM_hip_motor Jp2_COM_hip_motor JpEE_COM_hip_motor;...
                                            Jo1 Jo2 Jo3];
    
    P_COM_thigh = T0_COM_thigh(1:3,4);
    Jp1_COM_thigh = cross(Z0, P_COM_thigh-P0);
    Jp2_COM_thigh = cross(Z1, P_COM_thigh-P1);
    JpEE_COM_thigh = cross(Z2, P_COM_thigh-P2);
    Jacs.exo.left.hip_to_COM_thigh = [Jp1_COM_thigh Jp2_COM_thigh JpEE_COM_thigh;...   
                                            Jo1 Jo2 Jo3];
    
    P_COM_sup_thigh_cuff = T0_COM_sup_thigh_cuff(1:3,4);
    Jp1_COM_sup_thigh_cuff = cross(Z0, P_COM_sup_thigh_cuff-P0);
    Jp2_COM_sup_thigh_cuff = cross(Z1, P_COM_sup_thigh_cuff-P1);
    JpEE_COM_sup_thigh_cuff = cross(Z2, P_COM_sup_thigh_cuff-P2);
    Jacs.exo.left.hip_to_COM_sup_thigh_cuff = [Jp1_COM_sup_thigh_cuff Jp2_COM_sup_thigh_cuff JpEE_COM_sup_thigh_cuff;...
                                            Jo1 Jo2 Jo3];

    P_COM_inf_thigh_cuff = T0_COM_inf_thigh_cuff(1:3,4);
    Jp1_COM_inf_thigh_cuff = cross(Z0, P_COM_inf_thigh_cuff-P0);
    Jp2_COM_inf_thigh_cuff = cross(Z1, P_COM_inf_thigh_cuff-P1);
    JpEE_COM_inf_thigh_cuff = cross(Z2, P_COM_inf_thigh_cuff-P2);
    Jacs.exo.left.hip_to_COM_inf_thigh_cuff = [Jp1_COM_inf_thigh_cuff Jp2_COM_inf_thigh_cuff JpEE_COM_inf_thigh_cuff; ...
                                            Jo1 Jo2 Jo3];
    
    
    P_COM_knee_shell = T0_COM_knee_shell(1:3,4);
    Jp1_COM_knee_shell = cross(Z0, P_COM_knee_shell-P0);
    Jp2_COM_knee_shell = cross(Z1, P_COM_knee_shell-P1);
    JpEE_COM_knee_shell = cross(Z2, P_COM_knee_shell-P2);
    Jacs.exo.left.hip_to_COM_knee_shell = [Jp1_COM_knee_shell Jp2_COM_knee_shell JpEE_COM_knee_shell;...
                                            Jo1 Jo2 Jo3];

    P3 = T03(1:3,4);
    Z3 = T03(1:3,3);
    Jo4 = Z3;
    
    P_COM_knee_motor = T0_COM_knee_motor(1:3,4);
    Jp1_knee_motor =cross(Z0,P_COM_knee_motor-P0);
    Jp2_knee_motor =cross(Z1,P_COM_knee_motor-P1);
    Jp3_knee_motor =cross(Z2,P_COM_knee_motor-P2);
    Jp4_knee_motor =cross(Z3,P_COM_knee_motor-P3);
    Jacs.exo.left.hip_to_COM_knee_motor = [Jp1_knee_motor Jp2_knee_motor Jp3_knee_motor Jp4_knee_motor; ...
                                        Jo1 Jo2 Jo3 Jo4];
    
    P_COM_shank = T0_COM_shank(1:3,4);
    Jp1_COM_shank = cross(Z0, P_COM_shank-P0);
    Jp2_COM_shank = cross(Z1, P_COM_shank-P1);
    Jp3_COM_shank = cross(Z2, P_COM_shank-P2);
    JpEE_COM_shank = cross(Z3, P_COM_shank-P3);
    Jacs.exo.left.hip_to_COM_shank = [Jp1_COM_shank Jp2_COM_shank Jp3_COM_shank JpEE_COM_shank; ...
                                        Jo1 Jo2 Jo3 Jo4];
    
    P_COM_sup_shank_cuff = T0_COM_sup_shank_cuff(1:3,4);
    Jp1_COM_sup_shank_cuff = cross(Z0, P_COM_sup_shank_cuff-P0);
    Jp2_COM_sup_shank_cuff = cross(Z1, P_COM_sup_shank_cuff-P1);
    Jp3_COM_sup_shank_cuff = cross(Z2, P_COM_sup_shank_cuff-P2);
    JpEE_COM_sup_shank_cuff = cross(Z3, P_COM_sup_shank_cuff-P3);
    Jacs.exo.left.hip_to_COM_sup_shank_cuff = [Jp1_COM_sup_shank_cuff Jp2_COM_sup_shank_cuff Jp3_COM_sup_shank_cuff JpEE_COM_sup_shank_cuff; ...
                                                Jo1 Jo2 Jo3 Jo4];

    P_COM_inf_shank_cuff = T0_COM_inf_shank_cuff(1:3,4);
    Jp1_COM_inf_shank_cuff = cross(Z0, P_COM_inf_shank_cuff-P0);
    Jp2_COM_inf_shank_cuff = cross(Z1, P_COM_inf_shank_cuff-P1);
    Jp3_COM_inf_shank_cuff = cross(Z2, P_COM_inf_shank_cuff-P2);
    JpEE_COM_inf_shank_cuff = cross(Z3, P_COM_inf_shank_cuff-P3);
    Jacs.exo.left.hip_to_COM_inf_shank_cuff = [Jp1_COM_inf_shank_cuff Jp2_COM_inf_shank_cuff Jp3_COM_inf_shank_cuff JpEE_COM_inf_shank_cuff; ...
                                                Jo1 Jo2 Jo3 Jo4];

    P_COM_ankle_shell = T0_COM_ankle_shell(1:3,4);
    Jp1_COM_ankle_shell =cross(Z0,P_COM_ankle_shell-P0);
    Jp2_COM_ankle_shell =cross(Z1,P_COM_ankle_shell-P1);
    Jp3_COM_ankle_shell =cross(Z2,P_COM_ankle_shell-P2);
    Jp4_COM_ankle_shell =cross(Z3,P_COM_ankle_shell-P3);
    Jacs.exo.left.hip_to_COM_ankle_shell=[Jp1_COM_ankle_shell Jp2_COM_ankle_shell Jp3_COM_ankle_shell Jp4_COM_ankle_shell;...
                                           Jo1 Jo2 Jo3 Jo4];

    P4 = T04(1:3,4);
    Z4 = T04(1:3,3);
    Jo5 = Z4;
    
    P_COM_ankle_motor = T0_COM_ankle_motor(1:3,4);
    Jp1_COM_ankle_motor =cross(Z0,P_COM_ankle_motor-P0);
    Jp2_COM_ankle_motor =cross(Z1,P_COM_ankle_motor-P1);
    Jp3_COM_ankle_motor =cross(Z2,P_COM_ankle_motor-P2);
    Jp4_COM_ankle_motor =cross(Z3,P_COM_ankle_motor-P3);
    Jp5_COM_ankle_motor =cross(Z4,P_COM_ankle_motor-P4);
    Jacs.exo.left.hip_to_COM_ankle_motor=[Jp1_COM_ankle_motor Jp2_COM_ankle_motor Jp3_COM_ankle_motor Jp4_COM_ankle_motor Jp5_COM_ankle_motor;...
                                    Jo1 Jo2 Jo3 Jo4 Jo5];

    
    P_COM_a = T0_COM_a(1:3,4);
    Jp1_COM_a = cross(Z0, P_COM_a-P0);
    Jp2_COM_a = cross(Z1, P_COM_a-P1);
    Jp3_COM_a = cross(Z2, P_COM_a-P2);
    Jp4_COM_a = cross(Z3, P_COM_a-P3);
    JpEE_COM_a = cross(Z4, P_COM_a-P4);
    Jacs.exo.left.hip_to_COM_foot = [Jp1_COM_a Jp2_COM_a Jp3_COM_a Jp4_COM_a JpEE_COM_a; Jo1 Jo2 Jo3 Jo4 Jo5];
else
    Jacs=[];
end
end