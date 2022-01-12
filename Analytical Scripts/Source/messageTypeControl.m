% type control function
function []= messageTypeControl( canch, control)
 message_type_control=canMessage(71,false,6);
%  if control == 'Position'
%     control_type = 1;
%  elseif control == 'Stiffness'
%      control_type = 2;
%  elseif control == 'Torque'
%      control_type = 3;
%  elseif control == 'Deactivate'
%      control_type = 4;
%  elseif control == 'Stopped'
%      control_type = 5;
%  end
 control_type = control;
% control_type = control;
 message_type_control.Data=(hex2dec(ndec2hex([ control_type control_type...
                                          control_type  1 1 1],8)))';
 transmit(canch, message_type_control);
end