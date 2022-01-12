function [COM_foot_r, COM_foot_l] = calcHumanFootCOM(model)

m_foot2392_r = model.get_BodySet().get('calcn_r').getMass() + ...
               model.get_BodySet().get('toes_r').getMass();
m_foot2392_l = model.get_BodySet().get('calcn_l').getMass() + ...
               model.get_BodySet().get('toes_l').getMass();

COM_talus_r = osimVec3ToArray(model.get_BodySet().get('talus_r').getMassCenter());
COM_talus_l = osimVec3ToArray(model.get_BodySet().get('talus_l').getMassCenter());

talus_calcn_offset_r = osimVec3ToArray(model.getJointSet().get('subtalar_r').get_frames(0).get_translation());
calcn_toes_offset_r = osimVec3ToArray(model.getJointSet().get('mtp_r').get_frames(0).get_translation());
talus_calcn_offset_l = osimVec3ToArray(model.getJointSet().get('subtalar_l').get_frames(0).get_translation());
calcn_toes_offset_l = osimVec3ToArray(model.getJointSet().get('mtp_l').get_frames(0).get_translation());

COM_calcn_r_calcn_frame = osimVec3ToArray(model.get_BodySet().get('calcn_r').getMassCenter());
COM_calcn_r_talus_frame = COM_calcn_r_calcn_frame + talus_calcn_offset_r; 

COM_calcn_l_calcn_frame = osimVec3ToArray(model.get_BodySet().get('calcn_l').getMassCenter());
COM_calcn_l_talus_frame = COM_calcn_l_calcn_frame + talus_calcn_offset_l; 

COM_toes_r_toes_frame = osimVec3ToArray(model.get_BodySet().get('toes_r').getMassCenter());
COM_toes_r_talus_frame = talus_calcn_offset_r + calcn_toes_offset_r + COM_toes_r_toes_frame;

COM_toes_l_toes_frame = osimVec3ToArray(model.get_BodySet().get('toes_l').getMassCenter());
COM_toes_l_talus_frame = talus_calcn_offset_l + calcn_toes_offset_l + COM_toes_l_toes_frame;

COM_foot_r = (COM_talus_r*model.get_BodySet().get('talus_r').getMass() +...
                   COM_calcn_r_talus_frame*model.get_BodySet().get('calcn_r').getMass() + ...
                   COM_toes_r_talus_frame*model.get_BodySet().get('toes_r').getMass())/m_foot2392_r;
COM_foot_l = (COM_talus_l*model.get_BodySet().get('talus_l').getMass() +...
                   COM_calcn_l_talus_frame*model.get_BodySet().get('calcn_l').getMass() + ...
                   COM_toes_l_talus_frame*model.get_BodySet().get('toes_l').getMass())/m_foot2392_l;


end