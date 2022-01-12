function femur_length = getFemurLength(q_knee)
%make sure q_knee is in radians

y_offset_spline_x = [-2.0944 -1.22173 -0.523599 -0.349066 -0.174533 0.159149 2.0944];
y_offset_spline_y = [-0.4226 -0.4082 -0.399 -0.3976 -0.3966 -0.395264 -0.396];
y_offset_cs = csapi(y_offset_spline_x, y_offset_spline_y);

x_offset_spline_x = [-2.0944 -1.74533 -1.39626 -1.0472 -0.698132 -0.349066 -0.174533 0.197344 0.337395 0.490178 1.52146 2.0944];
x_offset_spline_y = [-0.0032 0.00179 0.00411 0.0041 0.00212 -0.001 -0.0031 -0.005227 -0.005435 -0.005574 -0.005435 -0.00525];
x_offset_cs = csapi(x_offset_spline_x,x_offset_spline_y);


y_length=fnval(y_offset_cs,q_knee);
x_length=fnval(x_offset_cs,q_knee);

femur_length = sqrt(x_length.^2+y_length.^2);

end