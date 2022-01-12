function Lengths = getModelLengths(model,XoR_or_H3_flag)
%% Get link lengths
% This does not work because of the spline curves
%femur_tibia_offset = osimVec3ToArray(model.getJointSet().get('knee_r').get_frames(0).get_translation()); 
% % Get distance between hip and knee based on knee angle (using splines as
% % defined in the model gait2392)
% %left and right splines are exactly the same
% x_offset_spline_x = [-2.0944 -1.74533 -1.39626 -1.0472 -0.698132 -0.349066 -0.174533 0.197344 0.337395 0.490178 1.52146 2.0944];
% x_offset_spline_y = [-0.0032 0.00179 0.00411 0.0041 0.00212 -0.001 -0.0031 -0.005227 -0.005435 -0.005574 -0.005435 -0.00525];
% x_offset_cs = csapi(x_offset_spline_x,x_offset_spline_y);
% 
% y_offset_spline_x = [-2.0944 -1.22173 -0.523599 -0.349066 -0.174533 0.159149 2.0944];
% y_offset_spline_y = [-0.4226 -0.4082 -0.399 -0.3976 -0.3966 -0.395264 -0.396];
% y_offset_cs = csapi(y_offset_spline_x, y_offset_spline_y);

% Get human lengths if model contains human model
if model.getJointSet.contains('subtalar_r')
    length_femur = 0.395; % value obtained from OpenSim documentation for knee at 0 deg
    
    knee_talus_offset_r = osimVec3ToArray(model.getJointSet().get('ankle_r').get_frames(0).get_translation());
    length_tibia_r = sqrt(knee_talus_offset_r(1)^2+knee_talus_offset_r(2)^2+knee_talus_offset_r(3)^2);
    knee_talus_offset_l = osimVec3ToArray(model.getJointSet().get('ankle_l').get_frames(0).get_translation());
    length_tibia_l = sqrt(knee_talus_offset_l(1)^2+knee_talus_offset_l(2)^2+knee_talus_offset_l(3)^2);
    
    Lengths.human.femur = length_femur; % the exact length of the left and right femur are calculated based on the knee angle and the spline curves
    Lengths.human.tibia_r = length_tibia_r;
    Lengths.human.tibia_l = length_tibia_l;
    
    talus_calcn_offset_r = osimVec3ToArray(model.getJointSet().get('subtalar_r').get_frames(0).get_translation());
    calcn_toes_offset_r = osimVec3ToArray(model.getJointSet().get('mtp_r').get_frames(0).get_translation());
    talus_calcn_offset_l = osimVec3ToArray(model.getJointSet().get('subtalar_l').get_frames(0).get_translation());
    calcn_toes_offset_l = osimVec3ToArray(model.getJointSet().get('mtp_l').get_frames(0).get_translation());
    
    COM_toes_r_toes_frame = osimVec3ToArray(model.get_BodySet().get('toes_r').getMassCenter());
    COM_toes_r_talus_frame = talus_calcn_offset_r + calcn_toes_offset_r + COM_toes_r_toes_frame*2; %multiply by 2 as a conservative approximation in order to avoid dripping due to insufficient foot clearance.
    COM_toes_l_toes_frame = osimVec3ToArray(model.get_BodySet().get('toes_l').getMassCenter());
    COM_toes_l_talus_frame = talus_calcn_offset_l + calcn_toes_offset_l + COM_toes_l_toes_frame*2;
    
    
    Lengths.human.foot_r = abs(COM_toes_r_talus_frame(1));
    Lengths.human.foot_l = abs(COM_toes_l_talus_frame(1));
end

% Get exo lengths if model contains exo model
if isequal(XoR_or_H3_flag,'XoR')
    length_thigh_offset_r = osimVec3ToArray(model.getJointSet().get('XoR_thigh_r_XoR_shank_r').get_frames(0).get_translation());
    Lengths.exo.thigh_r = sqrt(length_thigh_offset_r(1)^2+length_thigh_offset_r(2)^2+length_thigh_offset_r(3)^2);
    length_shank_offset_r = osimVec3ToArray(model.getJointSet().get('XoR_shank_r_XoR_foot_r').get_frames(0).get_translation());
    Lengths.exo.shank_r = sqrt(length_shank_offset_r(1)^2+length_shank_offset_r(2)^2+length_shank_offset_r(3)^2);
    
    length_thigh_offset_l = osimVec3ToArray(model.getJointSet().get('XoR_thigh_l_XoR_shank_l').get_frames(0).get_translation());
    Lengths.exo.thigh_l = sqrt(length_thigh_offset_l(1)^2+length_thigh_offset_l(2)^2+length_thigh_offset_l(3)^2);
    length_shank_offset_l = osimVec3ToArray(model.getJointSet().get('XoR_shank_l_XoR_foot_l').get_frames(0).get_translation());
    Lengths.exo.shank_l = sqrt(length_shank_offset_l(1)^2+length_shank_offset_l(2)^2+length_shank_offset_l(3)^2);
    
    Lengths.exo.foot_r = 0.19; %manually measured using Opensim GUI
    Lengths.exo.foot_l = 0.19; %manually measured using Opensim GUI
elseif isequal(XoR_or_H3_flag,'H3')
    length_thigh_offset_r = osimVec3ToArray(model.getJointSet().get('hip_motor_r_H3_thigh_r').get_frames(0).get_translation()) ...
                        + osimVec3ToArray(model.getJointSet().get('H3_thigh_r_H3_knee_shell_r').get_frames(0).get_translation());
    length_shank_offset_r = osimVec3ToArray(model.getJointSet().get('knee_motor_r_H3_shank_r').get_frames(0).get_translation()) ...
                        + osimVec3ToArray(model.getJointSet().get('H3_shank_r_H3_ankle_shell_r').get_frames(0).get_translation());
    
    Lengths.exo.thigh_r = abs(length_thigh_offset_r(2));
    Lengths.exo.shank_r = abs(length_shank_offset_r(2));
    Lengths.exo.foot_r = 0.22;%manually measured using CAD
    
    length_thigh_offset_l = osimVec3ToArray(model.getJointSet().get('hip_motor_l_H3_thigh_l').get_frames(0).get_translation()) ...
        + osimVec3ToArray(model.getJointSet().get('H3_thigh_l_H3_knee_shell_l').get_frames(0).get_translation());
    length_shank_offset_l = osimVec3ToArray(model.getJointSet().get('knee_motor_l_H3_shank_l').get_frames(0).get_translation()) ...
        + osimVec3ToArray(model.getJointSet().get('H3_shank_l_H3_ankle_shell_l').get_frames(0).get_translation());
    
    Lengths.exo.thigh_l =abs(length_thigh_offset_l(2));
    Lengths.exo.shank_l =abs(length_shank_offset_l(2));
    Lengths.exo.foot_l = 0.22; %manually measured using CAD
end
end