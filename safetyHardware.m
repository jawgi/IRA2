% Set up communication channel with arduino (hardware option 1)
% baudRate = 9600; % Baud rate
% comPort = 'COM3'; % com port
% arduino = serialport(comPort, baudRate); % arduino

% Set up controller communication (hardware option 2)
global id;
id = 1;
global joy;
joy = vrjoystick(id);

% Declare global variables
global estop; % overall status for if any estops are triggered
global estopDoor; %for estop at door
global estopRobot1; %for estop 1
global estopRobot2; %for estop 1
global estopCollection; %for estop 1
global doorOpen; %for door open sensor
global terminate; %end program

% Set all sensors to off initially
estop = 0;
estopDoor = 0; 
estopRobot1 = 0;
estopRobot2 = 0;
estopCollection = 0;
doorOpen = 0;
terminate = 0;

% Create a timer to check serial communication
tArd = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, ...
    'TimerFcn', @(~,~) checkSerial(arduino));

tCont = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, ...
    'TimerFcn', @(~,~) checkController(joy));

% Start the timer
start(tCont);

while(1)
    if terminate == 1
        error("Program terminated. Cya!");
    end
    if estop == 0
        disp("Program running");
    elseif estop == 1
        disp("Estop triggered, program pausing");
    end
    
    pause(1);
end
    
function checkSerial(serialObj)
    %global estop; % Declare global variable

    if serialObj.BytesAvailable > 0
        data = fgets(serialObj); % Read data from Arduino
        data = strtrim(data); % Remove whitespace

        % Display received signal
        disp(['Received: ', data]);

        % Conditional behavior based on the received message
        switch data
            case 'e1 triggered'
                disp('Emergency stop at door has been engaged.');
                estopDoor = 1; % Set E-stop
            case 'e1 released'
                disp('Emergency stop  at door has been released.');
                estopDoor = 0; % Set E-stop
            case 'e2 triggered'
                disp('Emergency stop at robot 1 has been engaged.');
                estopRobot1 = 1; % Set E-stop
            case 'e2 released'
                disp('Emergency stop  at robot 1 has been released.');
                estopRobot1 = 0; % Set E-stop
            case 'e3 triggered'
                disp('Emergency stop at robot 2 has been engaged.');
                estopRobot2 = 1; % Set E-stop
            case 'e3 released'
                disp('Emergency stop  at robot 2 has been released.');
                estopRobot2 = 0; % Set E-stop
            case 'e4 triggered'
                disp('Emergency stop at collection point has been engaged.');
                estopCollection = 1; % Set E-stop
            case 'e4 released'
                disp('Emergency stop at collection point has been released.');
                estopCollection = 0; % Set E-stop
            case 'door open sensor triggered'
                disp('Door has been opened.');
                doorOpen = 1; % Set E-stop
            case 'door open sensor released'
                disp('Door has been closed.');
                doorOpen = 0; % Set E-stop
        end

        estopVal = estopDoor + estopRobot1 + estopRobot2 + estopCollection + doorOpen;
        
        % If any estops are triggered, wait until reset to resume
        if estopVal > 0
            estop = 1;
        else
            estop = 0;
        end

        disp(["Estop value : ", estop]); % Display current E-stop value
    end
end


function checkController(joy)
global estop; % overall status for if any estops are triggered
global estopDoor; %for estop at door
global estopRobot1; %for estop 1
global estopRobot2; %for estop 1
global estopCollection; %for estop 1
global doorOpen; %for door open sensor
global terminate;

[axes, buttons, povs] = read(joy);
square = buttons(1);
cross = buttons(2);
circle = buttons(3);
triangle = buttons(4);
l1 = buttons(5);
l2 = buttons(7);
l3 = buttons(11);
r1 = buttons(6);
r2 = buttons(8);
r3 = buttons(12);
home = buttons(14);

    if square == 1 && estopDoor == 0
        disp('Emergency stop at door has been engaged.');
        estopDoor = 1;
    elseif triangle == 1 && estopDoor == 1
        disp('Emergency stop  at door has been released.');
        estopDoor = 0;
    elseif cross == 1 && estopRobot1 == 0
        disp('Emergency stop at robot 1 has been engaged.');
        estopRobot1 = 1;
    elseif circle == 1 && estopRobot1 == 1
        disp('Emergency stop  at robot 1 has been released.');
        estopRobot1 = 0;
    elseif l1 == 1 && estopRobot2 == 0
        disp('Emergency stop at robot 2 has been engaged.');
        estopRobot2 = 1;
    elseif l2 == 1 && estopRobot2 == 1
        disp('Emergency stop  at robot 2 has been released.');
        estopRobot2 = 0;
    elseif r1 == 1 && estopCollection == 0
        disp('Emergency stop at collection point has been engaged.');
        estopCollection = 1;
    elseif r2 == 1 && estopCollection == 1
        disp('Emergency stop at collection point has been released.');
        estopCollection = 0;
    elseif l3 == 1 && doorOpen == 0
        disp('Door has been opened.');
        doorOpen = 1;
    elseif r3 == 1 && doorOpen == 1
        disp('Door has been closed.');
        doorOpen = 0;
    
    % add && terminate == 0 to prevent excessive output if button is held/
    elseif home == 1 && terminate == 0
        terminate = 1;
        disp("Terminating");
    end

    estopVal = estopDoor + estopRobot1 + estopRobot2 + estopCollection + doorOpen;
        
    % If any estops are triggered, wait until reset to resume
    if estopVal > 0
        estop = 1;
    else
        estop = 0;
    end

    %disp(["Estop value : ", estop]); % Display current E-stop value

end
