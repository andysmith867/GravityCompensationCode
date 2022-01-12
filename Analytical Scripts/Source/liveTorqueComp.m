% function [hip_torque_right_int, knee_torque_right_int, ankle_torque_right_int] = liveTorqueComp...
%     (XX1, XX2, hip_right_angle_live, knee_right_angle_live, ankle_right_angle_live,  ...
%       hip_surfaces, knee_surfaces,ankle_surfaces, hip_angles,  ankle_angles)
%% Hip Torque Right
%notes

% just  "surfaces" which will be imported. This is the interpolated
% data and you need to find the necessary torque in this function. Then the
% interpolation should be done outside the live loop and then this function
% run inside the live loop. This will minimise time taken to solve for the
% necessary torque value.
hip_right_angle_live = 11;
knee_right_angle_live = 23;
ankle_right_angle_live = 10;
counter = 1;
hip_coord=ones(2,4);

for i = 1:length(hip_angles)

    if hip_right_angle_live >= hip_angles(i) && hip_right_angle_live <= hip_angles(i+1) ...
        || hip_right_angle_live<= hip_angles(i) && hip_right_angle_live >= hip_angles(i-1)
        
        hip_coord(counter,1) = hip_angles(i);
        
        values = cell2mat(hip_surfaces(i));
        

        for k= 1:length(XX1)
            if XX1(:,k) == knee_right_angle_live
                hip_coord(1:2,2) = k;
            end
        end
        for k= 1:(max(ankle_angles)-min(ankle_angles))+1
            if XX2(k,:) == ankle_right_angle_live
                hip_coord(1:2,3) = k;
            end
        end
        hip_coord(counter,4) = values(hip_coord(1,1)+1,hip_coord(1,2)+1);

        counter = counter +1;
%     interpolate between the two surface values
    end
end
x3 = [hip_coord(1,1) hip_coord(2,1)];
v = [hip_coord(1,4) hip_coord(2,4)];
xq = (hip_coord(1,1):1:hip_coord(2,1));
hipTorques = interpn(x3, v,xq, 'spline');

% find the torque value that corelates to the hip Angle

    for i = 1:length(xq)
        if hip_right_angle_live == xq(i)
            hip_torque_right_int = hipTorques(i);
            %disp(hipTorqueRightInt)
        end
    
    end

    
    %% knee Torque
counter_knee = 1;
knee_coord=ones(2,4);
for i = 1:length(hip_angles)

    if hip_right_angle_live >= hip_angles(i) && hip_right_angle_live <= hip_angles(i+1) ...
        || hip_right_angle_live<= hip_angles(i) && hip_right_angle_live >= hip_angles(i-1)
        knee_coord(counter_knee,1) = hip_angles(i);
        
        values = cell2mat(knee_surfaces(i));
        

        for k= 1:length(XX1)
            if XX1(:,k) == knee_right_angle_live
                knee_coord(1:2,2) = k;
            end
        end
        for k= 1:(max(ankle_angles)-min(ankle_angles))+1
            if XX2(k,:) == ankle_right_angle_live
                knee_coord(1:2,3) = k;
            end
        end
        knee_coord(counter_knee,4) = values(knee_coord(1,1)+1,knee_coord(1,2)+1);

        counter_knee = counter_knee +1;
%     interpolate between the two surface values
    end
end
x3 = [knee_coord(1,1) knee_coord(2,1)];
v = [knee_coord(1,4) knee_coord(2,4)];
xq = (knee_coord(1,1):1:knee_coord(2,1));
knee_torques = interpn(x3, v,xq, 'spline');

% find the torque value that corelates to the hip Angle

    for i = 1:length(xq)
        if hip_right_angle_live == xq(i)
            knee_torque_right_int = knee_torques(i);
            %disp(kneeTorqueRightInt)
        end
    
    end
    
%% Ankle Angle 


counter_ankle = 1;
ankle_coord=ones(2,4);
for i = 1:length(hip_angles)

    if hip_right_angle_live >= hip_angles(i) && hip_right_angle_live <= hip_angles(i+1) ...
        || hip_right_angle_live<= hip_angles(i) && hip_right_angle_live >= hip_angles(i-1)
        ankle_coord(counter_ankle,1) = hip_angles(i);
        
        ankle_values = cell2mat(ankle_surfaces(i));
        

        for k= 1:length(XX1)
            if XX1(:,k) == knee_right_angle_live
                ankle_coord(1:2,2) = k;
            end
        end
        for k= 1:(max(ankle_angles)-min(ankle_angles))+1
            if XX2(k,:) == ankle_right_angle_live
                ankle_coord(1:2,3) = k;
            end
        end
        ankle_coord(counter_ankle,4) = ankle_values(ankle_coord(1,1)+1,ankle_coord(1,2)+1);

        counter_ankle = counter_ankle +1;
%     interpolate between the two surface values
    end
end
x3 = [ankle_coord(1,1) ankle_coord(2,1)];
v = [ankle_coord(1,4) ankle_coord(2,4)];
xq = (ankle_coord(1,1):1:ankle_coord(2,1));
ankle_torques = interpn(x3, v,xq, 'spline');

% find the torque value that corelates to the hip Angle

    for i = 1:length(xq)
        if hip_right_angle_live == xq(i)
            ankle_torque_right_int = ankle_torques(i);
            %disp(ankleTorqueRightInt)
        end
    
    end

