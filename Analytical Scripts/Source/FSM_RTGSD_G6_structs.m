function [gait_state_new, change_detected] = FSM_RTGSD_G6_structs(FSM_Parameters, time_steps,i,force_state_change)

% TODO 
% - Allow going backwards

gait_state_old = FSM_Parameters.gait_state{end};


if ~exist('force_state_change', 'var')
    force_state_change=false;
end

heel_position_r = FSM_Parameters.heel_pos_r(1:i,:);
heel_position_l = FSM_Parameters.heel_pos_l(1:i,:);
toes_position_r = FSM_Parameters.toes_pos_r(1:i,:);
toes_position_l = FSM_Parameters.toes_pos_l(1:i,:);

if i>1
    heel_speed_l=diff(heel_position_l)./diff(time_steps(1:i));
    heel_speed_r=diff(heel_position_r)./diff(time_steps(1:i));
    toes_speed_r=diff(toes_position_r)./diff(time_steps(1:i));
    toes_speed_l=diff(toes_position_l)./diff(time_steps(1:i));
else
    heel_speed_l=[0 0 0];
    heel_speed_r=[0 0 0];
    toes_speed_l=[0 0 0];
    toes_speed_r=[0 0 0];
end
change_detected = 0;
mid_swing_thresh = 0.3;
speed_thresh = 0.05;

if i>1 && i-FSM_Parameters.gait_state_change(end)>FSM_Parameters.min_gait_state_index_duration   % the sign < should have been >
    if isequal(gait_state_old,'FSW_R') && (heel_speed_r(end,2)<-speed_thresh || force_state_change==true)
        disp('HS_R >>> DS_RF');
        gait_state_new='DS_RF';
        change_detected=1;
    elseif isequal(gait_state_old,'DS_RF') && (toes_speed_l(end,2)>speed_thresh || force_state_change==true)
        disp('TO_L >>> ISW_L');
        gait_state_new='ISW_L';
        change_detected=1;
    elseif isequal(gait_state_old, 'ISW_L') && (heel_position_l(end,1)>-mid_swing_thresh/2 || force_state_change==true) %y axis in trasformation is positive pointing in the posterior direction
        disp('MSW_L');
        gait_state_new='MSW_L';
        change_detected=1;
    elseif isequal(gait_state_old, 'MSW_L') && (heel_position_l(end,1)>mid_swing_thresh/2 || force_state_change==true) %y axis in trasformation is positive pointing in the posterior direction
        disp('FSW_L')
        gait_state_new='FSW_L';
        change_detected=1;
    elseif isequal(gait_state_old,'FSW_L') && (heel_speed_l(end,2)<-speed_thresh || force_state_change==true)
        disp('HS_L >>> DS_LF');
        gait_state_new='DS_LF';
        change_detected=1;
        % if isequal(gait_state,'DS_LF') && isequal(sum(Q_act(i,5)>Q_act([i-thresh:i-1],5)),thresh) %if data is noisy this will never be true
    elseif isequal(gait_state_old,'DS_LF') && (toes_speed_r(end,2)>speed_thresh || force_state_change==true)
        disp('TO_R >>> ISW_R')
        gait_state_new='ISW_R';
        change_detected=1;
    elseif isequal(gait_state_old, 'ISW_R') && (heel_position_r(end,1)>-mid_swing_thresh/2 || force_state_change==true) %y axis in trasformation is positive pointing in the posterior direction
        disp('MSW_R')
        gait_state_new='MSW_R';
        change_detected=1;
    elseif isequal(gait_state_old, 'MSW_R') && (heel_position_r(end,1)>mid_swing_thresh/2 || force_state_change==true) %y axis in trasformation is positive pointing in the posterior direction
        disp('FSW_R')
        gait_state_new='FSW_R';
        change_detected=1;
    end
end

if change_detected==0
    gait_state_new=gait_state_old;
end

end