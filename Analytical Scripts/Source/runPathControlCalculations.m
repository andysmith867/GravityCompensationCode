function [assistance, q_ref_r_i, q_ref_l_i, ind_r, ind_l]=runPathControlCalculations(Q_REF, q_act_r,q_act_l,k_exo,db_w)


    % Mapping based on Euclidean distance
    [~, ind_r] = getMapping(Q_REF.Q_ref_short(:,[1,3,5])*180*0.318309886183791, q_act_r);
    [~, ind_l] = getMapping(Q_REF.Q_ref_short(:,[2,4,6])*180*0.318309886183791, q_act_l);
    
    % Get joint error
    if db_w==0
        q_ref_r_i = Q_REF.Q_ref_short(ind_r,[1,3,5])*180*0.318309886183791;
        q_ref_l_i = Q_REF.Q_ref_short(ind_l,[2,4,6])*180*0.318309886183791;
    else
        q_ref_r_i = getMappingOnDb(Q_REF.Q_ref_short(ind_r,[1,3,5])*180*0.318309886183791,q_act_r,db_w);
        q_ref_l_i = getMappingOnDb(Q_REF.Q_ref_short(ind_l,[2,4,6])*180*0.318309886183791,q_act_l,db_w);
    end
    
    joint_error_r = q_ref_r_i - q_act_r;
    joint_error_l = q_ref_l_i - q_act_l;
    
    % Calculate assistive torques
    assistance = k_exo.*[joint_error_r, joint_error_l];
end