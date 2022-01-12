function [FSM_Param]=getGaitState(FSM_Param,Q_struct,dQ_struct,RTGSD,time_steps,i,Mov_Win_Param)

if ~exist('Mov_Win_Param','var')
    Mov_Win_Param.force_state_change=false;
end

if isequal(RTGSD,'min')
    [gait_state, FSM_Param.change_detected] = FSM_RTGSD_min_structs(Q_struct,dQ_struct,FSM_Param,i,Mov_Win_Param.force_state_change);
    FSM_Param.gait_state=[FSM_Param.gait_state gait_state];
elseif isequal(RTGSD, 'G6')
    [gait_state, FSM_Param.change_detected] = FSM_RTGSD_G6_structs(FSM_Param,time_steps,i,Mov_Win_Param.force_state_change);
    FSM_Param.gait_state=[FSM_Param.gait_state gait_state];
elseif isequal(RTGSD, 'mix')
    [gait_state, FSM_Param.change_detected] = FSM_RTGSD_minG6_structs(Q_struct,dQ_struct,FSM_Param,time_steps,i,Mov_Win_Param.force_state_change);
    FSM_Param.gait_state=[FSM_Param.gait_state gait_state];
end

end