%% Compensation Routine 
%  Please note that the "angles" variable should range from 45:-5 in the
%  second column. OpenSim will output 5:-45, double check before running 
%% Connect To CAN Bus
if exist('canch','var')
    stop(canch)
end
clear
clc

addpath('..\Source\')
import org.opensim.modeling.*
model = Model('..\Models\Participant1_H3.osim');
Inertias = getModelInertias(model,'H3');
Lengths = getModelLengths(model,'H3');
g = abs(model.getGravity().get(1));

%% Generate Angles
hip_angles_applied = -30:5:50;
knee_angles_applied = 40:5:60;
ankle_angles_applied = -30:10:30;

%% Initialise canch channel
% t = connectToTCPServer('192.168.10.2');
antiGravFlag= 1;
canch = canChannel('PEAK-System','PCAN_USBBUS1');
configBusSpeed(canch,1e6);
% allow the angle data to pass through the filter
filterAllowOnly(canch, 110, 'Standard');
start(canch)
%% Set Type Control and Return to Default position
messageTypeControl(canch, 1);
pause(0.01)
exo_angle_command=canMessage(72, false,6);
exo_angle_command.Data =(hex2dec(ndec2hex([0 0 0 0 0 0],8)))'; %set hip position depending on input
transmit(canch, exo_angle_command)
pause(0.01)
%% Set Torque Control And Initialise Some Stuff
% control = 'Torque';
messageTypeControl(canch,  3);
pause(0.01)

counter = 1;

applied_torque = [];
times = [];
Q_knee_r_stored =[];
loop_count = 0;


Q_act=struct('hip_adduction_r',[],'hip_adduction_l',[],'hip_rotation_r',[],'hip_rotation_l',[],'hip_flexion_r',[],'hip_flexion_l',[],'knee_angle_r',[],'knee_angle_l',[],'ankle_angle_r',[],'ankle_angle_l',[],...
    'H3_hip_adduction_r',[],'H3_hip_adduction_l',[],'H3_hip_rotation_r',[],'H3_hip_rotation_l',[],'H3_hip_flexion_r',[],'H3_hip_flexion_l',[],'H3_knee_angle_r',[],'H3_knee_angle_l',[],'H3_ankle_angle_r',[],'H3_ankle_angle_l',[]);

exo_torque_command=canMessage(73, false,6);

%% Real Time Loop For Gravity Copmensation With Data Collection (standing experiment)
% Will record the position, from this we can derive acceleration and
% velocity

flushinput(t)
while true
    if t.BytesAvailable > 0
        vicon = fread(t,1,'int8') ;
        if vicon == 1
            disp(num2str(vicon))
            break
        end
    end
end

t_start=tic;
%%
for i =1 :length(hip_angles_applied)
    for  j = 1:length(knee_angles_applied)
        for k=1:length(ankle_angles_applied)
            
            
            messageTypeControl(canch, 1)
            pause(0.01)
            grav_comp(counter,:) = [0 0 0 0 0 0];
            applied_torque(counter,:) = [0 0 0];
            t_end = toc(t_start);
            disp(t_end)
            times(counter,:) = t_end;
            counter = counter +1 ;

            exo_angle_command=canMessage(72, false,6);
            exo_angle_command.Data =(hex2dec(ndec2hex([hip_angles_applied(i) knee_angles_applied(j) ankle_angles_applied(k) 0 0 0],8)))'; %set hip position depending on input
            transmit(canch, exo_angle_command)
            pause(0.01)
            grav_comp(counter,:) = [0 0 0 0 0 0];
            applied_torque(counter,:) = [0 0 0];
            t_end = toc(t_start);
            disp(t_end)
            times(counter,:) = t_end;
            counter = counter +1 ;

            
            for t = 1:500-2
                if toc(t_start)<0.01
                    pause(0.011-toc(t_start))
                end
                % get angles and store
                t_start = tic;
                [Q_act] = getAngles_Qact(Q_act,antiGravFlag, canch, counter);
                Q_act.hip_adduction_r(counter)=0;
                Q_act.hip_adduction_l(counter)=0;
                Q_act.hip_rotation_r(counter)=0;
                Q_act.hip_rotation_l(counter)=0;
                
                Q_hip_stored.right(counter,:) = Q_act.hip_flexion_r(counter);
                Q_hip_stored.left(counter,:) = Q_act.hip_flexion_l(counter);
                Q_act.knee_angle_r=-Q_act.knee_angle_r;
                Q_knee_stored.right(counter,:) = Q_act.knee_angle_r(counter);
                Q_knee_stored.left(counter,:) = Q_act.knee_angle_l(counter);
                Q_ankle_stored.right(counter,:) = Q_act.ankle_angle_r(counter);
                Q_ankle_stored.left(counter,:) = Q_act.ankle_angle_l(counter);
                
                Q_act.H3_hip_flexion_r(counter) = Q_act.hip_flexion_r(counter);
                Q_act.H3_knee_angle_r(counter) = Q_act.knee_angle_r(counter);
                Q_act.H3_ankle_angle_r(counter)  = Q_act.ankle_angle_r(counter);
                Q_act.H3_hip_flexion_l(counter) = Q_act.hip_flexion_l(counter);
                Q_act.H3_knee_angle_l(counter) = Q_act.knee_angle_l(counter);
                Q_act.H3_ankle_angle_l(counter) = Q_act.ankle_angle_l(counter);
                Q_act.H3_hip_adduction_r(counter)=0;
                Q_act.H3_hip_adduction_l(counter)=0;
                Q_act.H3_hip_rotation_r(counter)=0;
                Q_act.H3_hip_rotation_l(counter)=0;
                
                
                if t==200 
                    % at 2 seconds, switch to gravity compensation
                    messageTypeControl(canch, 3)
                    pause(0.01)
                    grav_comp(counter,:) = [0 0 0 0 0 0];
                    applied_torque(counter,:) = [0 0 0];
                    t_end = toc(t_start);
                    disp(t_end)
                    times(counter,:) = t_end;
                    counter = counter +1 ;
                elseif t>200
                                       
% get jacobian and calculate gravity compensating torque
[Jacs, Transformations] = getH3Jacobians(model,Q_act,Inertias,Lengths,true,counter);
% calculate torque produced by mass and com for each link of exo and limb
% of human
taus_h_r = transpose(Jacs.exo.right.hip_to_COM_hip_motor)*[g*Inertias.exo.masses.hip_motor_r;0;0;0;0;0] + ...
    transpose(Jacs.exo.right.hip_to_COM_thigh)*[g*Inertias.exo.masses.H3_thigh_r;0;0;0;0;0] + ...
    transpose(Jacs.exo.right.hip_to_COM_sup_thigh_cuff)*[g*Inertias.exo.masses.H3_sup_thigh_girth_r;0;0;0;0;0] + ...
    transpose(Jacs.exo.right.hip_to_COM_inf_thigh_cuff)*[g*Inertias.exo.masses.H3_inf_thigh_girth_r;0;0;0;0;0] + ...
    transpose(Jacs.exo.right.hip_to_COM_knee_shell)*[g*Inertias.exo.masses.H3_knee_shell_r;0;0;0;0;0] + ...
    transpose(Jacs.exo.right.hip_to_COM_knee_motor(:,1:3))*[g*Inertias.exo.masses.knee_motor_r;0;0;0;0;0];
taus_h_l = transpose(Jacs.exo.left.hip_to_COM_hip_motor)*[g*Inertias.exo.masses.hip_motor_l;0;0;0;0;0] + ...
    transpose(Jacs.exo.left.hip_to_COM_thigh)*[g*Inertias.exo.masses.H3_thigh_l;0;0;0;0;0] + ...
    transpose(Jacs.exo.left.hip_to_COM_sup_thigh_cuff)*[g*Inertias.exo.masses.H3_sup_thigh_girth_l;0;0;0;0;0] + ...
    transpose(Jacs.exo.left.hip_to_COM_inf_thigh_cuff)*[g*Inertias.exo.masses.H3_inf_thigh_girth_l;0;0;0;0;0] + ...
    transpose(Jacs.exo.left.hip_to_COM_knee_shell)*[g*Inertias.exo.masses.H3_knee_shell_l;0;0;0;0;0];
taus_k_r = transpose(Jacs.exo.right.hip_to_COM_shank)*[g*Inertias.exo.masses.H3_shank_r;0;0;0;0;0] + ...
    transpose(Jacs.exo.right.hip_to_COM_sup_shank_cuff)*[g*Inertias.exo.masses.H3_sup_shank_girth_r;0;0;0;0;0] + ...
    transpose(Jacs.exo.right.hip_to_COM_inf_shank_cuff)*[g*Inertias.exo.masses.H3_inf_shank_girth_r;0;0;0;0;0] + ...
    transpose(Jacs.exo.right.hip_to_COM_ankle_shell)*[g*Inertias.exo.masses.H3_ankle_shell_r;0;0;0;0;0] + ...
    transpose(Jacs.exo.right.hip_to_COM_ankle_motor(:,1:4))*[g*Inertias.exo.masses.ankle_motor_r;0;0;0;0;0];
taus_k_l = transpose(Jacs.exo.left.hip_to_COM_knee_motor)*[g*Inertias.exo.masses.knee_motor_l;0;0;0;0;0] + ...
    transpose(Jacs.exo.left.hip_to_COM_shank)*[g*Inertias.exo.masses.H3_shank_l;0;0;0;0;0] + ...
    transpose(Jacs.exo.left.hip_to_COM_sup_shank_cuff)*[g*Inertias.exo.masses.H3_sup_shank_girth_l;0;0;0;0;0] + ...
    transpose(Jacs.exo.left.hip_to_COM_inf_shank_cuff)*[g*Inertias.exo.masses.H3_inf_shank_girth_l;0;0;0;0;0] + ...
    transpose(Jacs.exo.left.hip_to_COM_ankle_shell)*[g*Inertias.exo.masses.H3_ankle_shell_l;0;0;0;0;0];
taus_a_r = transpose(Jacs.exo.right.hip_to_COM_foot)*[g*Inertias.exo.masses.H3_foot_r;0;0;0;0;0];
taus_a_l = transpose(Jacs.exo.left.hip_to_COM_foot)*[g*Inertias.exo.masses.H3_foot_l;0;0;0;0;0] + ...
    transpose(Jacs.exo.left.hip_to_COM_ankle_motor)*[g*Inertias.exo.masses.ankle_motor_l;0;0;0;0;0];
% calculate hip torque
                    tau_h_r=taus_h_r(1)+taus_k_r(1)+taus_a_r(1);
                    tau_h_l=taus_h_l(1)+taus_k_l(1)+taus_a_l(1);
% calculate knee torque
                    tau_k_r=taus_k_r(4)+taus_a_r(4);
                    tau_k_l=taus_k_l(4)+taus_a_l(4);
% calculate ankle torque
                    tau_a_r=taus_a_r(5);
                    tau_a_l=taus_a_l(5);
                    
                    grav_comp(counter,:) = [tau_h_r,tau_h_l, tau_k_r,tau_k_l,tau_a_r,tau_a_l];
                    
                    hip_r_torque = round(tau_h_r);
                    knee_r_torque = round(tau_k_r);
                    ankle_r_torque = round(tau_a_r);

                    hip_l_torque =0;
                    knee_l_torque = 0;
                    ankle_l_torque = 0;
% save applied torques, convert from OpenSim frame of reference to Exo-H3
                    applied_torque(counter,:) = [hip_r_torque -1*knee_r_torque ankle_r_torque];
% transmit torques
                    exo_torque_command.Data =(hex2dec(ndec2hex([hip_r_torque -1*knee_r_torque ankle_r_torque ...
                        hip_l_torque knee_l_torque ankle_l_torque],8)))'; %set hip position depending on input
                    transmit(canch, exo_torque_command)
                    
                    t_end = toc(t_start);
                    disp(t_end)
                    times(counter,:) = t_end;
                    counter = counter +1 ;
                    
                else
                    grav_comp(counter,:) = [0 0 0 0 0 0];
                    applied_torque(counter,:) = [0 0 0];
                    
                    
                    t_end = toc(t_start);
                    disp(t_end)
                    times(counter,:) = t_end;
                    counter = counter +1 ;
                    
                end
            end
        end
    end
end


