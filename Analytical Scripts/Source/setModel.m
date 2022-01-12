function osim_model=setModel(weak_or_healthy_flag,XoR_or_H3_flag)
import org.opensim.modeling.*

% Weakened muscles of weak model have max_isometric_force reduced by ~50% 
% for the following muscles:
% - med_gas
% - lat_gas
% - bifemsh
% - bifemlh
% - iliacus
% - psoas
% - rect_fem
% - vast_lat
% - tib_ant


if (isequal(weak_or_healthy_flag,'healthy') || isequal(weak_or_healthy_flag,'uni-weak')) && isequal(XoR_or_H3_flag,'XoR')
    osim_model = Model('..\Models\XoR2392_gravity_on_muscles_on.osim');
elseif isequal(weak_or_healthy_flag,'weak')  && isequal(XoR_or_H3_flag,'XoR')
    osim_model = Model('..\Models\XoR2392_gravity_on_weakened.osim');
elseif isequal(weak_or_healthy_flag,'healthy')  && isequal(XoR_or_H3_flag,'H3')
    osim_model = Model('..\Models\H3_gait2392.osim');
elseif isequal(weak_or_healthy_flag,'weak')  && isequal(XoR_or_H3_flag,'H3')
    osim_model = Model('..\Models\H3_gait2392_weak.osim');
elseif isequal(weak_or_healthy_flag,'healthy')  && isempty(XoR_or_H3_flag)
    osim_model = Model('..\Models\gait2392v4_Millard.osim');
elseif isequal(weak_or_healthy_flag,'weak')  && isempty(XoR_or_H3_flag)
    osim_model = Model('..\Models\gait2392v4_weak.osim');
else
    error('weak_or_healthy_flag does not match any of the models available')
end
end