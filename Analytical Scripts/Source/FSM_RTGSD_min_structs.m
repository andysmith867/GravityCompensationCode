function [gait_state_new, change_detected] = FSM_RTGSD_min_structs(Q_act,dQ_act,FSM_Parameters,i,force_state_change)

% TODO
% - Allow going backwards

gait_state_old=FSM_Parameters.gait_state{end};

if ~exist('force_state_change', 'var')
    force_state_change=false;
end

change_detected=0;
mid_swing_thresh=0.3;

if i>1 && i-FSM_Parameters.gait_state_change(end)>FSM_Parameters.min_gait_state_index_duration
    if isequal(gait_state_old,'FSW_R') && (Q_act.knee_angle_r(i)<Q_act.knee_angle_r(i-1) || force_state_change==true)
        disp('HS_R >>> DS_RF');
        gait_state_new='DS_RF';
        change_detected=1;
    
    elseif isequal(gait_state_old,'DS_RF') && (dQ_act.knee_angle_l(i)>dQ_act.knee_angle_l(i-1) || force_state_change==true) 
        disp('TO_L >>> ISW_L');
        gait_state_new='ISW_L';
        change_detected=1;
    
    elseif isequal(gait_state_old, 'ISW_L') && (FSM_Parameters.heel_pos_l(i,1)>-mid_swing_thresh/2 || force_state_change==true) 
        disp('MSW_L');
        gait_state_new='MSW_L';
        change_detected=1;
    
    elseif isequal(gait_state_old, 'MSW_L') && (FSM_Parameters.heel_pos_l(i,1)>mid_swing_thresh/2 || force_state_change==true) 
        disp('FSW_L')
        gait_state_new='FSW_L';
        change_detected=1;

    elseif isequal(gait_state_old,'FSW_L') && (Q_act.knee_angle_l(i)<Q_act.knee_angle_l(i-1) || force_state_change==true)
        disp('HS_L >>> DS_LF');
        gait_state_new='DS_LF';
        change_detected=1;
    
    elseif isequal(gait_state_old,'DS_LF') && (dQ_act.knee_angle_r(i)>dQ_act.knee_angle_r(i-1) || force_state_change==true)
        disp('TO_R >>> ISW_R')
        gait_state_new='ISW_R';
        change_detected=1;
        
    elseif isequal(gait_state_old, 'ISW_R') && (FSM_Parameters.heel_pos_r(i,1)>-mid_swing_thresh/2 || force_state_change==true) %y axis in trasformation is positive pointing in the posterior direction
        disp('MSW_R')
        gait_state_new='MSW_R';
        change_detected=1;
    
    elseif isequal(gait_state_old, 'MSW_R') && (FSM_Parameters.heel_pos_r(i,1)>mid_swing_thresh/2 || force_state_change==true) %y axis in trasformation is positive pointing in the posterior direction
        disp('FSW_R')
        gait_state_new='FSW_R';
        change_detected=1;
    
    elseif isequal(gait_state_old, 'MSW_R') && (FSM_Parameters.heel_pos_r(i,1)<-mid_swing_thresh/2 || force_state_change==true) %y axis in trasformation is positive pointing in the posterior direction
        disp('GOING BACKWARDS - ISW_R')
        gait_state_new='ISW_R';
        change_detected=1;
    end
end

if change_detected==0
    gait_state_new=gait_state_old;
end

end