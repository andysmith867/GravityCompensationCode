%% things to improve
% This is a costly function in terms of time, is there a faster way to run
% the function whilst keeping the interpolation within an acceptable
% margin? Is it possible to combine the three for loops into one for loop
% to minimise computational expense?  

% Pause introduced to allow for stabilisation (27/09/21) may be helpfiul
% for an individual to catch the exo also. This may not be as much of an
% issue with the individual in an exo as human will act as a damper

% [a,b]=find(ismember(reference,[105 90 0],'rows')>0) for referecne table
% look up

function [right_hip_torqueCAN, right_knee_torqueCAN, right_ankle_torqueCAN, ...
    hip_angles, knee_angles, ankle_angles, reference] = calibrationRoutine(calibration_flag)
% 
% If activated by main script

if calibration_flag == 1
    
%create a loop counter to store the data

    n = 1;
    
%Connect to CANbus 

%     canChannelList

    canch = canChannel('PEAK-System','PCAN_USBBUS1');
    configBusSpeed(canch,1e6);
    
%Initialise the can channel and buzzer

    filterAllowOnly(canch, [54,55], 'Standard');
    start(canch)
    buzzerOn(canch)
% limb limits
    limbLimits(canch)
% Message x4c (76) (Percentage of assistance)
    prompt= ('On a scale of 1:100, what percentage of assistance is required? \n');
    percentage_assistance = input(prompt);
    assistanceLevel(canch, percentage_assistance)
%Prescribe the exo to be in position control
    control = 'Position';
    messageTypeControl(canch, control);
    
    pause(0.1) %sampling frequency 100Hz    
    


        TorqueID = 120;             %message ID, from Exo H3 handbook pg 66

%create vectors for maximum joint angles

        hip_angles = -30 : 10: 50;     %define hip angle increments in multiples of ten
        knee_angles = 0:10:60;     %define knee angle increments multiples of ten
        ankle_angles = -30:20:30;    %define ankle angle increments multiples of ten
        
%create a can message to receive the torque data

        buzzerOn(canch)
%iterate through programme of angles and joints
        counter = 1;
        
        for i = 1:length(hip_angles) 
            
% %start with hips as this is the base of the pendulum

            exo_hip_command=canMessage(72, false,6);
            exo_hip_command.Data =(hex2dec(ndec2hex([hip_angles(i) 0 0 0 0 0],8)))'; %set hip position depending on input
            transmitPeriodic(canch,exo_hip_command, 'On', 0.01) 
            pause(1) %introduced after first trial to minimise jitter
            
%for each hip angle iterate through knee angle joints

            for j = 1:length(knee_angles)        %set knee angle
                exo_knee_command = canMessage(72, false,6);
                exo_knee_command.Data = (hex2dec(ndec2hex( [hip_angles(i) knee_angles(j) 0 0 0 0],8)))'; % set knee angle depending on input
                transmitPeriodic(canch, exo_knee_command, 'On', 0.01) %transmit message
                
%for each knee and hip angle iterate through all ankle angles 

                for k = 1:length(ankle_angles)   %set ankle angles
                    exo_ankle_command=canMessage(72, false,6);
                    exo_ankle_command.Data = (hex2dec(ndec2hex( [ hip_angles(i) knee_angles(j) ankle_angles(k) 0 0 0 ],8)))'; %set ankle angle depending on the input    
                    transmitPeriodic(canch, exo_ankle_command, 'On', 0.01) 
                    
%save data for reference matrix

                    reference(counter,1) = hip_angles(i);
                    reference(counter,2) = knee_angles(j);
                    reference(counter,3) = ankle_angles(k);
                    
                    counter = counter + 1;
                    
                    pause(3)     %wait for stabilisation, is this enough?
%receive torque data

%                     message_torque = receive(canch, Inf, 'OutputFormat', 'timetable'); 
%                     dato = cell2mat(message_torque.Data);
%                     message_torque = canMessage(500,false,8);
%                     message_torque.Data = dato(end,:);
                    
%                     pause(2)
                    
%store torque data in relevant arrays
                   
                      
%                     right_hip_torqueCAN(n) = unpack(message_torque,0,8,'BigEndian','int32');
%                     right_knee_torqueCAN(n) = unpack(message_torque,8,8,'BigEndian','int32');
%                     right_ankle_torqueCAN(n) = unpack(message_torque,16,8,'BigEndian','int32');

                    [right_hip_torque, right_knee_torque, right_ankle_torque, ... 
                    left_hip_torque, left_knee_torque, left_ankle_torque] = getCalibrationTorques(canch) ;
                    
                    right_hip_torqueCAN(n) = right_hip_torque;
                    right_knee_torqueCAN(n) = right_knee_torque;
                    right_ankle_torqueCAN(n) =right_ankle_torque;

                    n = n + 1;
                    
                end
            end
        end
        % Return the Exo to the original position once calibration routine is
% complete

exo_ankle_command=canMessage(72, false,6);
exo_ankle_command.Data = (hex2dec(ndec2hex( [ 0 0 0 0 0 0],8)))'; %set ankle angle depending on the input 
transmit(canch, exo_ankle_command) 

%End transmission
        stop(canch)
else
    right_hip_torqueCAN=0;
    right_knee_torqueCAN=0;
    right_ankle_torqueCAN=0;
    hip_angles=0; 
    knee_angles=0;
    ankle_angles=0;
    reference=0;
end



    
    
end

%% Notes
