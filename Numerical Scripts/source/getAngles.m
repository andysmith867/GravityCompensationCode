function [hip_right_angle_live, knee_right_angle_live, ankle_right_angle_live, hip_left_angle_live, ...
    knee_left_angle_live, ankle_left_angle_live] = getAngles(antiGravFlag, canch)
if antiGravFlag == 1 
    
    messageAngle = receive(canch, Inf, 'OutputFormat', 'timetable'); 
    
    while isempty(messageAngle)
     messageAngle = receive(canch, Inf, 'OutputFormat', 'timetable');
     disp('Empty message')
     pause(0.01)
    end
    dato = cell2mat(messageAngle.Data);
                    
    messageAngle = canMessage(700,false,8);
    messageAngle.Data = dato(end,:);
                    
        
%     pause(0.01)          
%store torque data in relevant arrays
    hip_right_angle_live= unpack(messageAngle,0,8,'BigEndian','int32');
    knee_right_angle_live = unpack(messageAngle,8,8,'BigEndian','int32');
    ankle_right_angle_live = unpack(messageAngle,16,8,'BigEndian','int32');
    hip_left_angle_live = unpack(messageAngle,24,8,'BigEndian','int32');
    knee_left_angle_live = unpack(messageAngle,32,8,'BigEndian','int32');
    ankle_left_angle_live = unpack(messageAngle,40, 8,'BigEndian','int32');
elseif antiGravFlag == 0
    warning('Program disrupted  by user')
    return
end
