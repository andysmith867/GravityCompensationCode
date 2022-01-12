function [] = assistanceLevel(canch, percentage_assistance)
 
    message_percentage_assistance=canMessage(76,false,6);
    message_percentage_assistance.Data=(hex2dec(ndec2hex([percentage_assistance ...
        percentage_assistance percentage_assistance percentage_assistance percentage_assistance ...
        percentage_assistance],8)))';
    transmit(canch, message_percentage_assistance)
    pause(0.1)
end
    