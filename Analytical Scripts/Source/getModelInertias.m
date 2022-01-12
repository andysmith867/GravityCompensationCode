function [Inertias] = getModelInertias(model,XoR_or_H3_flag)
% model needs to be XOR_gait2392 OpenSim model or H3_gait2392

%% Get masses, centre of mass for exo and link lengths
for i=1:model.getBodySet.getSize
    body_name=model.getBodySet.get(i-1).getName.toCharArray';
    if isempty(XoR_or_H3_flag)
        Inertias.human.masses.(body_name)=model.getBodySet().get(i-1).getMass();
        Inertias.human.COMs.(body_name)=osimVec3ToArray(model.getBodySet().get(i-1).getMassCenter());
    else
        if contains(body_name,XoR_or_H3_flag) || contains(body_name,'motor')
            Inertias.exo.masses.(body_name)=model.getBodySet().get(i-1).getMass();
            Inertias.exo.COMs.(body_name)=osimVec3ToArray(model.getBodySet().get(i-1).getMassCenter());
        else
            Inertias.human.masses.(body_name)=model.getBodySet().get(i-1).getMass();
            Inertias.human.COMs.(body_name)=osimVec3ToArray(model.getBodySet().get(i-1).getMassCenter());
        end
    end
end

if model.get_BodySet.contains('talus_r')
    Inertias.human.masses.foot_r = model.get_BodySet().get('talus_r').getMass()+...
        model.get_BodySet().get('calcn_r').getMass() + ...
        model.get_BodySet().get('toes_r').getMass();
    Inertias.human.masses.foot_l = model.get_BodySet().get('talus_l').getMass()+...
        model.get_BodySet().get('calcn_l').getMass() + ...
        model.get_BodySet().get('toes_l').getMass();
    [Inertias.human.COMs.foot_r, Inertias.human.COMs.foot_l] = calcHumanFootCOM(model);
end                    


end


