function [applied_torque] = getJointTorques(XX1,XX2, surfaces, hip_angle, knee_angle,ankle_angle, hip_angles, ankle_angles)
counter = 1;
coord=ones(2,4);
for i = 1:length(hip_angles)
    
    if hip_angle >= hip_angles(i) && hip_angle <= hip_angles(i+1) ...
            || hip_angle<= hip_angles(i) && hip_angle >= hip_angles(i-1)
        coord(counter,1) = hip_angles(i);
        
        values = cell2mat(surfaces(i));
        
        
        for k= 1:length(XX1)
            if XX1(:,k) == knee_angle
                coord(1:2,2) = k;
            end
        end
        for k= 1:(max(ankle_angles)-min(ankle_angles))+1
            if XX2(k,:) == ankle_angle
                coord(1:2,3) = k;
            end
        end
        coord(counter,4) = values(coord(1,3),coord(1,2));
        
        counter = counter +1;
        %     interpolate between the two surface values
    end
end
x3 = [coord(1,1) coord(2,1)];
v = [coord(1,4) coord(2,4)];
xq = (coord(1,1):1:coord(2,1));
torques = interpn(x3, v,xq, 'spline');

% find the torque value that corelates to the hip Angle

for i = 1:length(xq)
    if hip_angle == xq(i)
        applied_torque = torques(i);
        %disp(kneeTorqueRightInt)
    end
    
end
end