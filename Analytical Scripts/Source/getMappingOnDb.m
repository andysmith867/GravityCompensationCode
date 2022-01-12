function q_ref_out=getMappingOnDb(q_ref_in,q_act,db_w)

vector_Qact_to_Qref = q_ref_in - q_act;
vector_size = norm(vector_Qact_to_Qref);
unit_vector = vector_Qact_to_Qref/vector_size;
q_ref_out = q_ref_in -(unit_vector*db_w);

end