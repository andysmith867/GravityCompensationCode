function FSM_Parameters = initialiseFSMVariables(time_steps,ref_or_act,min_gait_state_time_duration)

if ~exist('min_gait_state_time_duration','var')
    min_gait_state_time_duration=0;
end    

if isequal(ref_or_act,'act') || isequal(ref_or_act,'Act') || isequal(ref_or_act,'ACT')
    FSM_Parameters.heel_pos_r = zeros(length(time_steps),3);
    FSM_Parameters.heel_pos_l = zeros(length(time_steps),3);
    FSM_Parameters.toes_pos_r = zeros(length(time_steps),3);
    FSM_Parameters.toes_pos_l = zeros(length(time_steps),3);
    FSM_Parameters.ind_l = zeros(length(time_steps),1);
    FSM_Parameters.ind_r = zeros(length(time_steps),1);
    FSM_Parameters.ind_Q_act_assist_l=ones(length(time_steps),1);
    FSM_Parameters.ind_Q_act_assist_r=ones(length(time_steps),1);
    FSM_Parameters.gait_state_change = 1; % marks the index at which gait states are changed
    FSM_Parameters.change_detected = 0;
    FSM_Parameters.FSM_labels = {'HS_R - DS_R_F'; 'TO_L - ISW_L'; 'MSW_L';'FSW_L';'HS_L - DS_L_F'; 'TO_R - ISW_R'; 'MSW_R';'FSW_R';};
    FSM_Parameters.min_gait_state_index_duration=round(min_gait_state_time_duration/mean(diff(time_steps)));
elseif isequal(ref_or_act,'ref') || isequal(ref_or_act,'Ref') || isequal(ref_or_act,'REF')
    FSM_Parameters.heel_pos_r = zeros(length(time_steps),3);
    FSM_Parameters.heel_pos_l = zeros(length(time_steps),3);
    FSM_Parameters.toes_pos_r = zeros(length(time_steps),3);
    FSM_Parameters.toes_pos_l = zeros(length(time_steps),3);
    FSM_Parameters.gait_state_change = 1;
    FSM_Parameters.change_detected = 0;
    FSM_Parameters.change_count=0;
    FSM_Parameters.FSM_labels = {'HS_R - DS_R_F'; 'TO_L - ISW_L'; 'MSW_L';'FSW_L';'HS_L - DS_L_F'; 'TO_R - ISW_R'; 'MSW_R';'FSW_R';};
    FSM_Parameters.min_gait_state_index_duration=0;
else
    error('The flag ref_or_act should be a character array including either the letter ref or the letters act.')
end
end
