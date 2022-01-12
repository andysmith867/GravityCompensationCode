function setInitialPosH3(canch, target)
% Message x4B (75) (Min angles accepted)
messageMinAngles=canMessage(75,false,6);
messageMinAngles.Data=(hex2dec(ndec2hex([-25 0 -25 -25 0 -25],8)))';


% Message x50 (80) (Max Angles accepted)
messageMaxAngles=canMessage(80,false,6);
messageMaxAngles.Data=(hex2dec(ndec2hex([100 100 25 100 100 25],8)))';
 
% Message x4c (76) (Percentage of assistance)

messagePercentageAssistance=canMessage(76,false,6);
messagePercentageAssistance.Data=(hex2dec(ndec2hex([80 80 80 80 80 80],8)))';

% Message (81) (Exo command)
buzzer_ON=19;

messageCommandExo=canMessage(81,false,6); %6 is the number of bytes we will send (for this exo is always six)
messageCommandExo.Data = (hex2dec(ndec2hex([0 buzzer_ON 0 0 0 0],8)))'; % in the second byte you can use the bluetooth commands from table 21

%
messagePosSetPoint=canMessage(72,false,6);

% Message x47 (71) (Type of control)

messageTypeControl=canMessage(71,false,6);
PositionControl=1;
StiffnessControl=2;
TorqueControl=3;
MotorDisabled=4;
MotorStopped=5;

messageTypeControl.Data=(hex2dec(ndec2hex([ PositionControl PositionControl PositionControl...
                                            PositionControl PositionControl PositionControl],8)))';

%% First set to zero if not already at zero
% Exo goes automatically to zero initial configuration with these commands
start(canch)
    transmit(canch,messageCommandExo)
    pause(0.1)
    transmit(canch,messageMinAngles) % not necessary
    pause(0.1) % minimum pause is 0.01 because of the can-bus operating at 100Hz
    transmit(canch,messageMaxAngles) % not necessary
    pause(0.1)
    transmit(canch,messagePercentageAssistance) % not necessary
    pause(0.1)
    transmit(canch,messageTypeControl)% necessary
    pause(1)
    messagePosSetPoint.Data=hex2dec(ndec2hex(round(target),8))';
    transmit(canch,messagePosSetPoint)
    pause(0.1)
stop(canch)


end