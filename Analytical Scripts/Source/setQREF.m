function [Q_REF,dQ_REF]=setQREF(Q_REF,dQ_REF,IND,init_gait_state)

IND_fieldnames=fieldnames(IND);

for i=1:length(IND_fieldnames)
    if isequal(IND_fieldnames{i},init_gait_state{1}) && i<length(IND_fieldnames)
        Q_REF.(IND_fieldnames{i}) = Q_REF.Q_ref_expanded([[IND.(IND_fieldnames{i}):end],[1:IND.(IND_fieldnames{i+1})]],:);  %final swing of right foot
        dQ_REF.(IND_fieldnames{i}) = dQ_REF.dQ_ref_expanded([[IND.(IND_fieldnames{i}):end],[1:IND.(IND_fieldnames{i+1})]],:);  %final swing of right foot
    elseif isequal(IND_fieldnames{i},init_gait_state{1}) && i==length(IND_fieldnames)
        Q_REF.(IND_fieldnames{i}) = Q_REF.Q_ref_expanded([[IND.(IND_fieldnames{i}):end],[1:IND.(IND_fieldnames{1})]],:);  %final swing of right foot
        dQ_REF.(IND_fieldnames{i}) = dQ_REF.dQ_ref_expanded([[IND.(IND_fieldnames{i}):end],[1:IND.(IND_fieldnames{1})]],:);  %final swing of right foot
    elseif ~isequal(IND_fieldnames{i},init_gait_state{1}) && i<length(IND_fieldnames)
        Q_REF.(IND_fieldnames{i}) = Q_REF.Q_ref_expanded([IND.(IND_fieldnames{i}):IND.(IND_fieldnames{i+1})],:);  %mid-swing of left foot
        dQ_REF.(IND_fieldnames{i}) = dQ_REF.dQ_ref_expanded([IND.(IND_fieldnames{i}):IND.(IND_fieldnames{i+1})],:);  %mid-swing of left foot
    elseif ~isequal(IND_fieldnames{i},init_gait_state{1}) && i==length(IND_fieldnames)
        Q_REF.(IND_fieldnames{i}) = Q_REF.Q_ref_expanded([IND.(IND_fieldnames{i}):IND.(IND_fieldnames{1})],:);  %mid-swing of left foot
        dQ_REF.(IND_fieldnames{i}) = dQ_REF.dQ_ref_expanded([IND.(IND_fieldnames{i}):IND.(IND_fieldnames{1})],:);  %mid-swing of left foot
    end

end