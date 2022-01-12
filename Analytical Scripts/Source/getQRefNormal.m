function [Q_REF, dQ_REF] = getQRefNormal(xor,XoR_or_H3_flag,control_data,RTGSD,plot_flag)
run_alone=false;

if ~exist('plot_flag','var')
    plot_flag=false;
end

if run_alone==true
    clear;clc;
    import org.opensim.modeling.*
    control_data = Data('..\MotionFiles\CMC_AndreasAvgGait\3DGaitModel2392_controls.sto');
    RTGSD='mix';
    XoR_or_H3_flag='H3';
    xor=setModel('healthy', XoR_or_H3_flag);
end

% Normal = Data('..\MotionFiles\normal_adjusted_fullbody_w_db.mot');
Normal = Data('..\MotionFiles\AndreasAvgGait.mot');
%granularity = length(control_data.getTimesteps)*10;
granularity = length(control_data.getTimesteps);
time_Normal = Normal.getColumn('time')';
time_Normal_expanded = stretchVector(time_Normal,granularity);


ankle_angle_l = Normal.getColumn('ankle_angle_l')*pi/180; % +ve = dorsiflexion
knee_angle_l = Normal.getColumn('knee_angle_l')*pi/180;
hip_flexion_l = Normal.getColumn('hip_flexion_l')*pi/180;

ankle_angle_r = Normal.getColumn('ankle_angle_r')*pi/180; % +ve = dorsiflexion
knee_angle_r = Normal.getColumn('knee_angle_r')*pi/180;
hip_flexion_r = Normal.getColumn('hip_flexion_r')*pi/180;

q_ref_2392_hip_add_l = Normal.getColumn('hip_adduction_l')*pi/180;
q_ref_2392_hip_add_r = Normal.getColumn('hip_adduction_r')*pi/180;
q_ref_2392_hip_rot_l = Normal.getColumn('hip_rotation_l')*pi/180;
q_ref_2392_hip_rot_r = Normal.getColumn('hip_rotation_r')*pi/180; 

q_ref_2392_hip_add_l_expanded = stretchVector(q_ref_2392_hip_add_l,granularity)';
q_ref_2392_hip_add_r_expanded = stretchVector(q_ref_2392_hip_add_r,granularity)';
q_ref_2392_hip_rot_l_expanded = stretchVector(q_ref_2392_hip_rot_l,granularity)';
q_ref_2392_hip_rot_r_expanded = stretchVector(q_ref_2392_hip_rot_r,granularity)';


% radius_db = stretchVector(Normal.getColumn('w_db_hip_l'), granularity);
% radius_fes = stretchVector(Normal.getColumn('w_fes_hip_l'), granularity);
radius_db = ones(granularity,1)*2;
radius_fes= radius_db*2;

Q_ref = [hip_flexion_r, hip_flexion_l, knee_angle_r, knee_angle_l, ankle_angle_r, ankle_angle_l];

dQ_ref = zeros(length(time_Normal),size(Q_ref,2));
for i=1:size(Q_ref,2)
    dQ_ref(2:end,i) = diff(Q_ref(:,i))./diff(time_Normal)';
end
dQ_ref(1,:)=dQ_ref(2,:);

dq_ref_2392_hip_rot_l = diff(q_ref_2392_hip_rot_l)./diff(time_Normal)';
dq_ref_2392_hip_rot_r = diff(q_ref_2392_hip_rot_r)./diff(time_Normal)';
dq_ref_2392_hip_add_l = diff(q_ref_2392_hip_add_l)./diff(time_Normal)';
dq_ref_2392_hip_add_r = diff(q_ref_2392_hip_add_r)./diff(time_Normal)';

dq_ref_2392_hip_add_l_expanded = stretchVector(dq_ref_2392_hip_add_l,granularity)';
dq_ref_2392_hip_add_r_expanded = stretchVector(dq_ref_2392_hip_add_r,granularity)';
dq_ref_2392_hip_rot_l_expanded = stretchVector(dq_ref_2392_hip_rot_l,granularity)';
dq_ref_2392_hip_rot_r_expanded = stretchVector(dq_ref_2392_hip_rot_r,granularity)';

Q_ref_expanded = zeros (granularity, size(Q_ref,2));
dQ_ref_expanded = zeros (granularity, size(dQ_ref,2));

for i = 1:size(Q_ref,2)
    Q_ref_expanded(:,i)=stretchVector(Q_ref(:,i),granularity);
    dQ_ref_expanded(:,i)=stretchVector(dQ_ref(:,i),granularity);
end
Q_REF.hip_flexion_r=Q_ref_expanded(:,1);
Q_REF.hip_flexion_l=Q_ref_expanded(:,2);
Q_REF.knee_angle_r=Q_ref_expanded(:,3);
Q_REF.knee_angle_l=Q_ref_expanded(:,4);
Q_REF.ankle_angle_r=Q_ref_expanded(:,5);
Q_REF.ankle_angle_l=Q_ref_expanded(:,6);
Q_REF.hip_add_r=q_ref_2392_hip_add_r_expanded;
Q_REF.hip_add_l=q_ref_2392_hip_add_l_expanded;
Q_REF.hip_rot_r=q_ref_2392_hip_rot_r_expanded;
Q_REF.hip_rot_l=q_ref_2392_hip_rot_l_expanded;

dQ_REF.hip_flexion_r=dQ_ref_expanded(:,1);
dQ_REF.hip_flexion_l=dQ_ref_expanded(:,2);
dQ_REF.knee_angle_r=dQ_ref_expanded(:,3);
dQ_REF.knee_angle_l=dQ_ref_expanded(:,4);
dQ_REF.ankle_angle_r=dQ_ref_expanded(:,5);
dQ_REF.ankle_angle_l=dQ_ref_expanded(:,6);
dQ_REF.hip_add_r=dq_ref_2392_hip_add_r_expanded;
dQ_REF.hip_add_l=dq_ref_2392_hip_add_l_expanded;
dQ_REF.hip_rot_r=dq_ref_2392_hip_rot_r_expanded;
dQ_REF.hip_rot_l=dq_ref_2392_hip_rot_l_expanded;


%% Define End Effector position and speed
Lengths = getModelLengths(xor,XoR_or_H3_flag);
FSM_Param_Ref = initialiseFSMVariables(time_Normal_expanded,'Ref');
% FSM_Param_Ref.gait_state={'FSW_R'};
FSM_Param_Ref.gait_state={'DS_RF'};
FSM_Param_Ref.change_count8_flag=false;

heel_position_ref_r = zeros(size(Q_ref,1),3);
heel_position_ref_l = zeros(size(Q_ref,1),3);
heel_position_ref_r_expanded = zeros(granularity,3);
heel_position_ref_l_expanded = zeros(granularity,3);
heel_speed_ref_r_expanded = zeros(granularity,3);
heel_speed_ref_l_expanded = zeros(granularity,3);

toes_position_ref_r = zeros(size(Q_ref,1),3);
toes_position_ref_l = zeros(size(Q_ref,1),3);
toes_position_ref_r_expanded = zeros(granularity,3);
toes_position_ref_l_expanded = zeros(granularity,3);
toes_speed_ref_r_expanded = zeros(granularity,3);
toes_speed_ref_l_expanded = zeros(granularity,3);

for i=1:size(Q_ref,1)
    femur_r=getFemurLength(Q_ref(:,3));
    femur_l=getFemurLength(Q_ref(:,4));
    
    [~,T_ha_l,T_h_toes_l]=Transformations_2392(q_ref_2392_hip_add_l(i), q_ref_2392_hip_rot_l(i), Q_ref(i,2),femur_l(1), Q_ref(i,4),Lengths.human.tibia_l, Q_ref(i,6),Lengths.human.foot_l);
    heel_position_ref_l(i,:) = [-T_ha_l(2,4) T_ha_l(1,4) T_ha_l(3,4)];
    toes_position_ref_l(i,:) = [-T_h_toes_l(2,4) T_h_toes_l(1,4) T_h_toes_l(3,4)];

    [~,T_ha_r,T_h_toes_r]=Transformations_2392(q_ref_2392_hip_add_r(i), q_ref_2392_hip_rot_r(i), Q_ref(i,1),femur_r(1), Q_ref(i,3),Lengths.human.tibia_r, Q_ref(i,5),Lengths.human.foot_r);
    heel_position_ref_r(i,:) = [-T_ha_r(2,4) T_ha_r(1,4) T_ha_r(3,4)];
    toes_position_ref_r(i,:) = [-T_h_toes_r(2,4) T_h_toes_r(1,4) T_h_toes_r(3,4)];
end

heel_speed_ref_r=diff(heel_position_ref_r)./diff(time_Normal)';
heel_speed_ref_l=diff(heel_position_ref_l)./diff(time_Normal)';
toes_speed_ref_r=diff(toes_position_ref_r)./diff(time_Normal)';
toes_speed_ref_l=diff(toes_position_ref_l)./diff(time_Normal)';

for i=1:size(heel_position_ref_r,2)
    heel_position_ref_l_expanded(:,i) = stretchVector(heel_position_ref_l(:,i),granularity)';
    heel_position_ref_r_expanded(:,i) = stretchVector(heel_position_ref_r(:,i),granularity)';
    heel_speed_ref_l_expanded(:,i) = stretchVector(heel_speed_ref_l(:,i),granularity)';
    heel_speed_ref_r_expanded(:,i) = stretchVector(heel_speed_ref_r(:,i),granularity)';
    toes_position_ref_l_expanded(:,i) = stretchVector(toes_position_ref_l(:,i),granularity)';
    toes_position_ref_r_expanded(:,i) = stretchVector(toes_position_ref_r(:,i),granularity)';
    toes_speed_ref_l_expanded(:,i) = stretchVector(toes_speed_ref_l(:,i),granularity)';
    toes_speed_ref_r_expanded(:,i) = stretchVector(toes_speed_ref_r(:,i),granularity)';
end

FSM_Param_Ref.heel_pos_r=heel_position_ref_r_expanded;
FSM_Param_Ref.heel_pos_l=heel_position_ref_l_expanded;
FSM_Param_Ref.heel_speed_r=heel_speed_ref_r_expanded;
FSM_Param_Ref.heel_speed_l=heel_speed_ref_l_expanded;
FSM_Param_Ref.toes_pos_r=toes_position_ref_r_expanded;
FSM_Param_Ref.toes_pos_l=toes_position_ref_l_expanded;
FSM_Param_Ref.toes_speed_r=toes_speed_ref_r_expanded;
FSM_Param_Ref.toes_speed_l=toes_speed_ref_l_expanded;

%% Run FSM
% Get the pose of the legs at key
% points in the gait cycle, e.g. toe off

for i=1:size(Q_ref_expanded,1)

    if FSM_Param_Ref.change_count~=8
        FSM_Param_Ref=getGaitState(FSM_Param_Ref,Q_REF,dQ_REF,RTGSD,time_Normal_expanded',i);
    elseif FSM_Param_Ref.change_count==8 && FSM_Param_Ref.change_count8_flag==false
        disp('All Gait Phases have been detected!!')
        FSM_Param_Ref.change_count8_flag=true;
    end
    
    if FSM_Param_Ref.change_detected==1
        FSM_Param_Ref.gait_state_change=[FSM_Param_Ref.gait_state_change, i];
    end
    
    if isequal(FSM_Param_Ref.gait_state{end},'DS_RF') && FSM_Param_Ref.change_detected==1 
        HS_r = [time_Normal_expanded(i) Q_ref_expanded(i,:)];
        FSM_Param_Ref.change_count=FSM_Param_Ref.change_count+1;
        FSM_Param_Ref.change_detected=0;
    elseif isequal(FSM_Param_Ref.gait_state{end},'ISW_L') && FSM_Param_Ref.change_detected==1
        ISW_l = [time_Normal_expanded(i) Q_ref_expanded(i,:)];
        FSM_Param_Ref.change_count=FSM_Param_Ref.change_count+1;
        FSM_Param_Ref.change_detected=0;
    elseif isequal(FSM_Param_Ref.gait_state{end},'MSW_L') && FSM_Param_Ref.change_detected==1
        MSW_l= [time_Normal_expanded(i) Q_ref_expanded(i,:)];
        FSM_Param_Ref.change_count=FSM_Param_Ref.change_count+1;
        FSM_Param_Ref.change_detected=0;
    elseif isequal(FSM_Param_Ref.gait_state{end},'FSW_L') && FSM_Param_Ref.change_detected==1
        FSW_l =[time_Normal_expanded(i) Q_ref_expanded(i,:)];
        FSM_Param_Ref.change_count=FSM_Param_Ref.change_count+1;
        FSM_Param_Ref.change_detected=0;
    elseif isequal(FSM_Param_Ref.gait_state{end},'DS_LF') && FSM_Param_Ref.change_detected==1
        HS_l=[time_Normal_expanded(i) Q_ref_expanded(i,:)];
        FSM_Param_Ref.change_count=FSM_Param_Ref.change_count+1;
        FSM_Param_Ref.change_detected=0;
    elseif isequal(FSM_Param_Ref.gait_state{end},'ISW_R') && FSM_Param_Ref.change_detected==1
        ISW_r=[time_Normal_expanded(i) Q_ref_expanded(i,:)];
        FSM_Param_Ref.change_count=FSM_Param_Ref.change_count+1;
        FSM_Param_Ref.change_detected=0;
    elseif isequal(FSM_Param_Ref.gait_state{end},'MSW_R') && FSM_Param_Ref.change_detected==1
        MSW_r=[time_Normal_expanded(i) Q_ref_expanded(i,:)];
        FSM_Param_Ref.change_count=FSM_Param_Ref.change_count+1;
        FSM_Param_Ref.change_detected=0;
    elseif isequal(FSM_Param_Ref.gait_state{end},'FSW_R') && FSM_Param_Ref.change_detected==1
        FSW_r=[time_Normal_expanded(i) Q_ref_expanded(i,:)];
        FSM_Param_Ref.change_count=FSM_Param_Ref.change_count+1;
        FSM_Param_Ref.change_detected=0;
    end    
end

%%
% Use those poses to ge the index in the reference trajectory with that
% pose
% These indices are based on the trajectory of the left leg. for simplicity
% we use the pose of the left leg for a key event to find the index in Q_ref_expanded that
% is the poses of the left leg is most similar
[~, ind_HS_r] = getMapping(Q_ref_expanded(:,[2,4,6]), HS_r([3,5,7]));

[~, ind_HS_l] = getMapping(Q_ref_expanded(:,[2,4,6]), HS_l([3,5,7]));

[~, ind_MSW_l] = getMapping(Q_ref_expanded(:,[2,4,6]), MSW_l([3,5,7]));

[~, ind_FSW_l] = getMapping(Q_ref_expanded(:,[2,4,6]), FSW_l([3,5,7]));

[~, ind_TO_r] = getMapping(Q_ref_expanded(:,[2,4,6]), ISW_r([3,5,7]));

[~, ind_TO_l] = getMapping(Q_ref_expanded(:,[2,4,6]), ISW_l([3,5,7]));

[~, ind_MSW_r] = getMapping(Q_ref_expanded(:,[2,4,6]), MSW_r([3,5,7]));

[~, ind_FSW_r] = getMapping(Q_ref_expanded(:,[2,4,6]), FSW_r([3,5,7]));

IND=struct('DS_LF',ind_HS_l, 'ISW_R',ind_TO_r,'MSW_R',ind_MSW_r,'FSW_R',ind_FSW_r,...
           'DS_RF',ind_HS_r, 'ISW_L',ind_TO_l,'MSW_L',ind_MSW_l,'FSW_L',ind_FSW_l);

Q_REF.Q_ref_expanded=Q_ref_expanded;
dQ_REF.dQ_ref_expanded=dQ_ref_expanded;
Q_REF.time = time_Normal_expanded;
        
[Q_REF,dQ_REF]=setQREF(Q_REF,dQ_REF,IND,FSM_Param_Ref.gait_state);

% Q_REF.SW_L = [Q_REF.ISW_L;Q_REF.MSW_L;Q_REF.FSW_L];
% Q_REF.SW_R = [Q_REF.ISW_R;Q_REF.MSW_R;Q_REF.FSW_R];
%% Plot
if plot_flag==true
    gait_events=figure();
    gait_events.WindowState = 'maximized';
    plotRefTraj(Q_REF,3,3)
    
    %%
    line_fig=figure('Position',[50 50 1500 700]);
    line=plot3(Q_ref_expanded(:,2), Q_ref_expanded(:,4), Q_ref_expanded(:,6));
    grid off;
    xlabel('Hip flexion angle (deg)','FontSize',16)
    ylabel('Knee flexion angle (deg)','FontSize',16)
    zlabel('Ankle flexion angle (deg)','FontSize',16)
    % title('Reference kinematic joint trajectory for left leg','FontSize',18)
    
    %%
    tube_fig=figure('Position',[50 50 1500 700]);
    ax1=axes;
    FontSizeMultiplier=2;
    ref_plot=plot3(Q_ref_expanded(:,2)*180/pi, Q_ref_expanded(:,4)*180/pi, Q_ref_expanded(:,6)*180/pi, 'k','LineWidth',2,'DisplayName','${\bf x}_{ref}$');
    hold on
    enlarge=1.4;
    xlimit=xlim;
    ylimit=ylim;
    zlimit=zlim;
    xlim([xlimit(1)*enlarge xlimit(2)*enlarge*0.8]);
    ylim([ylimit(1)*enlarge*0.8 ylimit(2)*enlarge]);
    zlim([zlimit(1)*enlarge zlimit(2)*enlarge]);
    xlimit=xlim;
    ylimit=ylim;
    zlimit=zlim;
    
    ax1.XLim=xlimit;
    ax1.YLim=ylimit;
    ax1.ZLim=zlimit;
    colour1='gray';
    colour2='copper';
    tube2=tubeplot_2axes([(Q_ref_expanded(:,2))'*180/pi Q_ref_expanded(1,2)*180/pi; (Q_ref_expanded(:,4))'*180/pi Q_ref_expanded(1,4)*180/pi; (Q_ref_expanded(:,6))'*180/pi Q_ref_expanded(1,6)*180/pi], radius_fes, 16,0.01,ax1,colour1,xlimit,ylimit,zlimit,FontSizeMultiplier,'FES Band');
    tube1=tubeplot_2axes([Q_ref_expanded(:,2)'*180/pi Q_ref_expanded(1,2)*180/pi; (Q_ref_expanded(:,4))'*180/pi Q_ref_expanded(1,4)*180/pi; (Q_ref_expanded(:,6))'*180/pi Q_ref_expanded(1,6)*180/pi], radius_db, 16,0.01,ax1,colour2,xlimit,ylimit,zlimit,FontSizeMultiplier,'Dead Band');
    %     tubeplot([(Q_ref_expanded(:,2))' Q_ref_expanded(1,2); (Q_ref_expanded(:,4))' Q_ref_expanded(1,4); (Q_ref_expanded(:,6))' Q_ref_expanded(1,6)], radius_fes*pi/180, 16,0.01);
    set(get(ax1,'XLabel'),'String','Hip angle (deg)','Interpreter','latex')
    set(get(ax1,'YLabel'),'String','Knee angle (deg)','Interpreter','latex')
    set(get(ax1,'ZLabel'),'String','Ankle angle (deg)','Interpreter','latex')
%     set(get(ax1,'Title'),'String','Left leg')
    set(ax1,'FontSize',10*FontSizeMultiplier,'TickLabelInterpreter','latex')

    pause(0.01)
    legend([ref_plot, tube1,tube2],'FontSize',10*FontSizeMultiplier,'Location','northeast','interpreter','latex');
    results_path='C:\Users\andre\Desktop\PhD\Code\MyCode\Images_Videos';
    exportgraphics(gcf,[results_path, '\DualTunnel.eps'],'Resolution',600)
    saveas(gcf,[results_path, '\DualTunnel.png'])
    saveas(gcf,[results_path, '\DualTunnel.fig'])

    %% plot q_ref Hybrid Path
    point=[0.45,-0.95,0];
    path_fig=figure();
    path_fig.WindowState='maximized';
    plot3(Q_ref(:,2),Q_ref(:,4),Q_ref(:,6),'k','LineWidth',3)
    hold on
    plot3(point(1),point(2),point(3),'kx','LineWidth',4,'MarkerSize',30)
    text(point(1)+0.02,point(2),point(3),'${\bf q}_{act}$','interpreter','latex','FontSize',70)

    q_diff=Q_ref(:,[2,4,6])-point;  
    dist = sqrt(q_diff(:,1).^2+q_diff(:,2).^2+q_diff(:,3).^2);
    [~, ind] = min(dist);
    plot3(Q_ref(ind,2),Q_ref(ind,4),Q_ref(ind,6),'kx','LineWidth',4,'MarkerSize',30)
    text(Q_ref(ind,2)-0.055,Q_ref(ind,4),Q_ref(ind,6)+0.02,'${\bf q}_{ref}$','interpreter','latex','FontSize',70)

    plot3([Q_ref(ind,2) point(1)],[Q_ref(ind,4) point(2)],[Q_ref(ind,6) point(3)],'k','LineWidth',1)
    %% 
    figure
    plot(dQ_ref_expanded)
end

end

