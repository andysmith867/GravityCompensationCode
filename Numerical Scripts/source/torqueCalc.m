 function [hip_surfaces, knee_surfaces, ankle_surfaces, XX1, XX2]= torqueCalc(hip_angles, knee_angles, ankle_angles, ...
     hipCAN, kneeCAN, ankleCAN) 
%% Notes

% 
% load('hip.mat'); %load data which can be used to test the script
% load('hipAngles.mat');
% load('kneeAngles.mat');
% load('ankleAngle');
% load('knee.mat');
% load('ankle.mat');
% % % 
% % %set some arbitrary angle set
% 
% hipAngleLive = 32; 
% kneeAngleLive = 21;
% ankleAngleLive = 12;

figure

%create a mash of the ankle and knee coordinates

x1(:,1) = knee_angles; 
x2(:,1) = ankle_angles;
[X1,X2] = meshgrid(x1,x2); 

% create for loop which creates a vector used as the query knee joints in
% the interpolation.

for i=0:max(knee_angles)
    xx1(i+1+i*(max(ankle_angles)-min(ankle_angles)):(i+1)*(max(ankle_angles)...
        -min(ankle_angles))+i+1,1)=i*ones((max(ankle_angles)...
        -min(ankle_angles))+1,1);
end

% create vector for query points for ankle angles 

xx2(:,1) = repmat(min(ankle_angles):max(ankle_angles),1,max(knee_angles)+1);

% using query point vectors, create a grid used for %mesh plotting

[XX1,XX2]=meshgrid(min(knee_angles):max(knee_angles),min(ankle_angles):max(ankle_angles));

% some intialisation stuff

%%figure
counter = 1;

% iterate through the full length of data points, and interpolate for the
% hip, ankle or knee torque.

for i = 1:length(hip_angles)
    
    torque=[];
    
    for j = 1:length(knee_angles)
        torque(:,j) = hipCAN(:,1+(j-1)*length(ankle_angles)+(i-1)*...
            length(ankle_angles)*length(knee_angles):3+(j-1)*...
            length(ankle_angles)+(i-1)*length(ankle_angles)*length(knee_angles));
    end

    %complete interpolation of surfaces
    interped = interpn(x1, x2, torque', xx1, xx2, 'cubic');
    
    % record the interpolated data
    
    % bug whereby this for only works if min knee angle is 0 degrees
    for k=0:max(knee_angles)-min(knee_angles)
        values(1:max(ankle_angles)-min(ankle_angles)+1,k+1)=...
            interped(k+1+k*((max(ankle_angles)-min(ankle_angles))):...
            (k+1)*(max(ankle_angles)-min(ankle_angles))+k+1,1,:);
        hip_surfaces{i}=values;
        
    end
    % identify the desired torque value from the %mesh surfaces
    counter =counter+1;
    %plot the %mesh
    mesh(XX1,XX2,values)
    hold on
    %         end
end

    xlabel('KneeAngle (degrees)')
    ylabel('Ankle Angle (degrees)')
    zlabel('Torque(Nm)')
    title('Hip Interaction Torques for Relevant Hip Angles')
 %% Knee Torque 


% some intialisation stuff
figure
kneeCoord = [];
counter = 1;

% iterate throguh the full length of data points, and interpolate for the
% hip, ankle or knee torque.

    for i = 1:length(hip_angles)
    
        torque=[];
        
            for j = 1:length(knee_angles) 
                torque(:,j) = kneeCAN(:,1+(j-1)*length(ankle_angles)+(i-1)*...
                    length(ankle_angles)*length(knee_angles):3+(j-1)*...
                    length(ankle_angles)+(i-1)*length(ankle_angles)*length(knee_angles));   
            end
            %complete interpolation of surfaces
            interped = interpn(x1, x2, torque', xx1, xx2, 'cubic');
            
            % record the interpolated data
            for k=0:max(knee_angles)
                values(1:max(ankle_angles)-min(ankle_angles)+1,k+1)=...
                    interped(k+1+k*((max(ankle_angles)-min(ankle_angles))):...
                    (k+1)*(max(ankle_angles)-min(ankle_angles))+k+1,1,:);
                knee_surfaces{i}=values;

            end
            % identify the desired torque value from the %mesh surfaces
            counter =counter+1;
            %plot the %mesh
            mesh(XX1,XX2,values)
            hold on
%         end
    end

    xlabel('KneeAngle (degrees)')
    ylabel('Ankle Angle (degrees)')
    zlabel('Torque(Nm)')
    title('Knee Interaction Torques for Relevant Hip Angles')
%% Ankle Torque

% some intialisation stuff
figure
counter = 1;

% iterate throguh the full length of data points, and interpolate for the
% hip, ankle or knee torque.

    for i = 1:length(hip_angles)
    
        torque=[];
        
            for j = 1:length(knee_angles) 
                torque(:,j) = ankleCAN(:,1+(j-1)*length(ankle_angles)+(i-1)*...
                    length(ankle_angles)*length(knee_angles):3+(j-1)*...
                    length(ankle_angles)+(i-1)*length(ankle_angles)*length(knee_angles));   
            end
            %complete interpolation of surfaces
            interped = interpn(x1, x2, torque', xx1, xx2, 'cubic');
            
            % record the interpolated data
            for k=0:max(knee_angles)
                values(1:max(ankle_angles)-min(ankle_angles)+1,k+1)=...
                    interped(k+1+k*((max(ankle_angles)-min(ankle_angles))):...
                    (k+1)*(max(ankle_angles)-min(ankle_angles))+k+1,1,:);
                ankle_surfaces{i}=values;
            end
            % identify the desired torque value from the %mesh surfaces
            counter =counter+1;
            %plot the %mesh
            mesh(XX1,XX2,values)
            hold on
%         end
    end

    xlabel('KneeAngle (degrees)')
    ylabel('Ankle Angle (degrees)')
    zlabel('Torque(Nm)')
    title('Ankle Interaction Torques for Relevant Hip Angles')
