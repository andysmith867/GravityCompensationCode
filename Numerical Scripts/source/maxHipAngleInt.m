nction [hip_torque_int] = maxHipAngleInt(XX1,XX2, hip_torque_values,...
    knee_angle_live, ankle_angle_live)
    XX1 = XX1(1,:);
    data_for_interp(1:2,1) = find(XX1 == knee_angle_live);
    XX2 = XX2(:,1);
    data_for_interp(1:2,2) = find(XX2 == ankle_angle_live);
    hip_torque_int =hip_torque_values(data_for_interp(1,2),data_for_interp(1,1));
    fprintf('Warning: No torque data for current hip angle, gravity compensation efficiency reduced');
    
end