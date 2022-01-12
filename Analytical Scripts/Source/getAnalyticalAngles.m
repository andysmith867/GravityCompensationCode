function [angles]=getAnalyticalAngles(hip_angles,knee_angles,ankle_angles)

%         hip_angles = -30 : 1:105;     %define hip angle increments in multiples of ten
%         knee_angles = 5:-1:-105;     %define knee angle increments multiples of ten
%         ankle_angles = -30:1:30;    %define ankle angle increments multiples of ten
        
%create a can message to receive the torque data

 
%iterate through programme of angles and joints
        counter = 1;
        
        for i = 1:length(hip_angles) 
            
% %start with hips as this is the base of the pendulum

%for each hip angle iterate through knee angle joints

            for j = 1:length(knee_angles)        %set knee angle
               
                
%for each knee and hip angle iterate through all ankle angles 

                for k = 1:length(ankle_angles)   %set ankle angles

%save data for reference matrix

                    reference(counter,1) = hip_angles(i);
                    reference(counter,2) = knee_angles(j);
                    reference(counter,3) = ankle_angles(k);
                    
                    counter = counter + 1
                    angles = reference;

                      
%                     right_hip_torqueCAN(n) = unpack(message_torque,0,8,'BigEndian','int32');
%                     right_knee_torqueCAN(n) = unpack(message_torque,8,8,'BigEndian','int32');
%                     right_ankle_torqueCAN(n) = unpack(message_torque,16,8,'BigEndian','int32')
                    
                end
            end
        end
end

%% Notes
