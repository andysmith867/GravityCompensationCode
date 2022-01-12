hip_angles = 0;
knee_angles = -105:1:5;
ankle_angles = -30:1:30;
counter = 1;
for i = 1:length(hip_angles)
    
    % %start with hips as this is the base of the pendulum
    
    
    %for each hip angle iterate through knee angle joints
    
    for j = 1:length(knee_angles)        %set knee angle
        
        
        for k = 1:length(ankle_angles)   %set ankle angles
            
            %save data for reference matrix
            
            reference(counter,1) = hip_angles(i);
            reference(counter,2) = knee_angles(j);
            reference(counter,3) = ankle_angles(k);
            
            counter = counter + 1;
            
            %receive torque data
            
         
            
            %                     pause(2)
            
            %store torque data in relevant arrays
            
        end
    end
end