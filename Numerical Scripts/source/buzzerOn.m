function []= buzzerOn(canch)
    buzzer_ON =19;
    buzz=canMessage(81,false,6); %6 is the number of bytes we will send (for this exo is always six)
    buzz.Data = (hex2dec(ndec2hex([0 buzzer_ON 0 0 0 0],8)))'; % in the second byte you can use the bluetooth commands from table 21
    transmit(canch, buzz)
end
