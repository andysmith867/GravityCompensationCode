function equilibriumExo(model, state ,XoR_or_H3_flag)
 
%    % Create a copy of the input model
%     model = org.opensim.modeling.Model();
%     model.assign(input_model);
%     model.initSystem();
    
    % Access coordinate set
    coordinates = model.getCoordinateSet();
    
    % Joints & bushing values we wish to get/set
    if isequal(XoR_or_H3_flag,'XoR')
        exo_joints = {'XoR_hip_adduction_r', 'XoR_hip_flexion_r', ...
            'XoR_knee_angle_r', 'XoR_hip_adduction_l', 'XoR_ankle_angle_r', ...
            'XoR_hip_flexion_l', 'XoR_knee_angle_l', 'XoR_ankle_angle_l'};
        bushings = {'XoR_thigh_r_bushing', 'XoR_shank_r_bushing', ...
            'XoR_foot_r_bushing', 'XoR_thigh_l_bushing', ...
            'XoR_shank_l_bushing', 'XoR_foot_l_bushing'};
        human_joints = {'hip_adduction_r', 'hip_flexion_r', 'knee_angle_r', ...
        'ankle_angle_r', 'hip_adduction_l', 'hip_flexion_l', 'knee_angle_l', ...
        'ankle_angle_l'};
    elseif isequal(XoR_or_H3_flag,'H3')
        exo_joints = {'H3_hip_flexion_r', 'H3_hip_flexion_l', ... 
                'H3_knee_angle_r', 'H3_knee_angle_l', ...
                'H3_ankle_angle_r', 'H3_ankle_angle_l'};
        bushings = {'sup_thigh_r_bushing', 'sup_thigh_l_bushing', ...
            'inf_thigh_r_bushing', 'inf_thigh_l_bushing', ...
            'sup_shank_r_bushing', 'sup_shank_l_bushing',...
            'inf_shank_r_bushing', 'inf_shank_l_bushing',...
            'foot_r_bushing', 'foot_l_bushing'};
        human_joints = {'hip_flexion_r', 'hip_flexion_l', 'knee_angle_r', ...
             'knee_angle_l', 'ankle_angle_r', 'ankle_angle_l'};
    else
        warning('Unrecognised exo model.')
    end
    n_bushings = length(bushings);
    n_joints = length(exo_joints);

    % Construct an initial guess for the optimal exo joints - the human
    % joint values
    x0 = zeros(n_joints, 1);
    for k = 1:n_joints
        x0(k) = coordinates.get(human_joints{k}).getValue(state);
    end
    
    % Use fminsearch to find the exo equilibrium position
    %options = optimset('PlotFcns', @optimplotfval);
    %x = fminsearch(@evaluate, x0, options);
    x = fminsearch(@evaluate, x0);
    
    % Use the fminsearch result to set the coordinates of the actual model
    coordinates = model.updCoordinateSet();
    for k = 1:length(exo_joints)
        coordinates.get(exo_joints{k}).setValue(state, x(k));
    end
    
    function result = evaluate(joint_values)
        
        % Set each of the joints in turn
        for i = 1:n_joints
            coordinates.get(exo_joints{i}).setValue(state, joint_values(i));
        end
        
        % Realise the stage to acceleration phase
        model.realizeAcceleration(state);
        
        % Compute the sum of squared bushing forces
        force_set = model.getForceSet();
        result = 0;
        for i = 1:n_bushings
            bushing_forces = force_set.get(bushings{i}).getRecordValues(state);
            for j = 0:bushing_forces.getSize() - 1
                %result = result + (bushing_forces.get(j))^2;
                result = result + abs(bushing_forces.get(j));
            end
        end
        
    end
    
end