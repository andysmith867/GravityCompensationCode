function [error, ind]=getMapping(Q_ref,Q_act)
% size(Q_ref) should be (N,3)
% size(Q_act) should be (1,3) or (3,1)

[r,~]=size(Q_ref);
if size(Q_act,1)<size(Q_act,2)
    Q_act=repmat(Q_act,r,1);
else 
    Q_act=repmat(Q_act,1,r)';
end
q_diff = Q_ref - Q_act;
dist = sqrt(q_diff(:,1).*q_diff(:,1)+q_diff(:,2).*q_diff(:,2)+q_diff(:,3).*q_diff(:,3));
[error, ind] = min(dist);

end