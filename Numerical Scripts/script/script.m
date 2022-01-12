%% Part 1: Calibration Routine
%% Ensure calibrationRoutine, torqueCalc, liveTorque and getAngles are added to your path
addpath('C:\Users\andre\Desktop\PhD\AndysCode\AntiGrav\Analytical Scripts\Source')
if exist('canch','var')
    stop(canch)
end
clear
clc

% clear
% clc
% %create a user input to start the routine
% 
% t = connectToTCPServer('192.168.10.2');
addpath('functions')
prompt = 'Would you like to begin calibration routine? 1 = yes, 0 = no \n';
calibration_flag =  input(prompt);

% %call the calibration routine function
% % calibration_runtime = tic;
% tic
[hipCAN, kneeCAN, ankleCAN, hip_angles, knee_angles, ankle_angles, reference] = calibrationRoutine(calibration_flag);

% toc(calibration_runtime);
%save the calibration routine data 
if calibration_flag
right_torquesCAN(:,1) = ankleCAN;
right_torquesCAN(:,2) = kneeCAN;
right_torquesCAN(:,3) = hipCAN;

end
%create a Microsoft Excel file for each torque
fprintf('Calibration routine complete \n');

% xlswrite('rightTorques.xlsx', rightTorquesCAN);
% toc(saveDataTime);
%
hip_angles_applied = -30:5:50;
knee_angles_applied = 30:5:60;
ankle_angles_applied = -30:10:30;

%% Part 2 : Gravity Compensation With Experimental Data Capture (standing experiment)
prompt='Would you like to apply gravity compensation routine? y = 1, n = 0 \n';
anti_grav_flag = input(prompt);
prompt = 'On a scale of 1:10, what percentage of gravity compensation is required? \n';
assistance_level = (input(prompt)/10);


% connect to can channel

canch = canChannel('PEAK-System','PCAN_USBBUS1');
configBusSpeed(canch,1e6);

% Only allow angle data to be retrieved

filterAllowOnly(canch, [110], 'Standard');

start(canch)

% Put exo into torque control

torque_control = 3;
message_type_control=canMessage(71,false,6);
PositionControl=1;                          %for different types of control, vary the integer - review H3 handbook

ankle_angles=ankleAngles;
hip_angles=hipAngles;
knee_angles=kneeAngles;

% interpolate for knee and ankle torques

[hip_surfaces, knee_surfaces, ankle_surfaces, XX1, XX2]= torqueCalc(hip_angles, knee_angles, ankle_angles, ...
    hipCAN, kneeCAN, ankleCAN); % this calls the torque Calc in the functions folder not the main directory
%create loop  for programme
pause(2)
close all


hip_torques_stored=[];
knee_torques_stored=[];
ankle_torques_stored=[];
times=[];
counter = 1;
% trigger

% flushinput(t) 
while true
    if t.BytesAvailable > 0
        vicon = fread(t,1,'int8') ;
        if vicon == 1
            disp(num2str(vicon))
            break
        end
    end
end
% antigrav flag

t_start = tic;
for i = 1:length(hip_angles_applied)
    for j = 1:length(knee_angles_applied)
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
            
            
            % retrieve angles from exoskeleton
            
            %     getAngleTime = tic;
            
            %get live angles
            for t = 1:500-2
                if toc(t_start)<0.01
                    pause(0.011-toc(t_start))
                end
                
                t_start = tic;
                
                [hip_angle_right_live, knee_angle_right_live, ankle_angle_right_live, hip_angle_left_live, ...
                    knee_angle_left_live, ankle_angle_left_live] = getAngles(anti_grav_flag, canch);
                
                hip_angles_stored.right(counter,:) = hip_angle_right_live;
                knee_angles_stored.right(counter,:) = knee_angle_right_live;
                ankle_angles_stored.right(counter,:) = ankle_angle_right_live;
                
                hip_angles_stored.left(counter,:) = hip_angle_left_live;
                knee_angles_stored.left(counter,:) = knee_angle_left_live;
                ankle_angles_stored.left(counter,:) = ankle_angle_left_live;
                
                % use live angles to interpolate between the hip torques
                %      [hip_torque_right_int, knee_torque_right_int, ankle_torque_right_int, hip_torque_left_int, ...
                %     knee_torque_left_int, ankle_torque_left_int]= liveTorque...
                %     (XX1, XX2, hip_angle_right_live, knee_angle_right_live, ankle_angle_right_live, hip_angle_left_live, ...
                %     knee_angle_left_live, ankle_angle_left_live, hip_surfaces, knee_surfaces, ...
                %     ankle_surfaces, hip_angles,  ankle_angles);
                
                if t==200
                    messageTypeControl(canch, 3)
                    pause(0.01)
                    grav_comp(counter,:) = [0 0 0 0 0 0];
                    applied_torque(counter,:) = [0 0 0];
                    t_end = toc(t_start);
                    disp(t_end)
                    times(counter,:) = t_end;
                    counter = counter +1 ;
                elseif t>200
                    [hip_right_torque] = (XX1,XX2, hip_surfaces, hip_angle_right_live, ...
                        knee_angle_right_live,ankle_angle_right_live, hip_angles, ankle_angles);
                    
                    [knee_right_torque] = getJointTorques(XX1,XX2, knee_surfaces, hip_angle_right_live, ...
                        knee_angle_right_live, ankle_angle_right_live,hip_angles, ankle_angles);
                    
                    [ankle_right_torque] = getJointTorques(XX1,XX2, ankle_surfaces, hip_angle_right_live,...
                        knee_angle_right_live,ankle_angle_right_live, hip_angles, ankle_angles);
                    
                    
                    
                    
                    [hip_left_torque] = getJointTorques(XX1,XX2, hip_surfaces, hip_angle_left_live,...
                        knee_angle_left_live,ankle_angle_left_live, hip_angles, ankle_angles);
                    
                    [knee_left_torque] = getJointTorques(XX1,XX2, knee_surfaces, hip_angle_left_live, ...
                        knee_angle_left_live, ankle_angle_left_live,hip_angles, ankle_angles);
                    
                    [ankle_left_torque] = getJointTorques(XX1,XX2, ankle_surfaces, hip_angle_left_live,...
                        knee_angle_left_live, ankle_angle_left_live,hip_angles, ankle_angles);
                    
                 
                    
                    % apply torques from interpolation
                    exoTorqueCommand=canMessage(73, false,6);
                    exoTorqueCommand.Data = (hex2dec(ndec2hex( [ ceil(assistance_level*round(hip_right_torque)) ...
                        ceil(assistance_level*round(knee_right_torque)) ceil(assistance_level*round(ankle_right_torque)) ...
                        0 0 0],8)))'; %set torque depending on the input
                    transmit(canch, exoTorqueCommand)
                    hip_torques_stored(counter,1)=hip_right_torque;
                    knee_torques_stored(counter,2)=knee_right_torque;
                    ankle_torques_stored(counter,3)=ankle_right_torque;
                    %pray and hope it works
                    t_end = toc(t_start);
                    times(counter,:) = t_end;
                    counter = counter +1;
                else
                    hip_torques_stored(counter,1)=0;
                    knee_torques_stored(counter,2)=0;
                    ankle_torques_stored(counter,3)=0;
                    t_end = toc(t_start);
                    times(counter,:) = t_end;
                    counter = counter +1;
                end
            end
        end
    end
end
