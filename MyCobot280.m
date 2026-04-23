classdef MyCobot280 < handle
    % MyCobot280 - MATLAB class for serial UART communication with myCobot 280 M5 robotic arm
    %
    % This class implements the communication protocol for controlling the
    % myCobot 280 M5 robot via serial UART interface.
    %
    % Usage:
    %   myc = MyCobot280('COM13');
    %   myc = MyCobot280('COM13', 'BaudRate', 115200, 'Debug', true);
    %   myc = MyCobot280('/dev/ttyUSB0', 'Timeout', 5, 'MaxRetries', 3);
    %
    % Options:
    %   'BaudRate'     - Serial baud rate (default: 115200)
    %   'Timeout'      - Read timeout in seconds (default: 5)
    %   'Debug'        - Print TX/RX bytes to console (default: false)
    %   'MaxRetries'   - Retry count on communication failure (default: 3)
    %   'CommandDelay' - Minimum delay between commands in seconds (default: 0.05)
    %
    % Author: Siddharth Vaghela
    % Date: 2026
    
    properties (Access = private)
        serialPort      % Serial port object
        portName        % Serial port name
        baudRate        % Baud rate (115200)
        timeout         % Communication timeout in seconds
        debugMode       % Enable/disable debug output
        maxRetries      % Number of retries for failed commands
        commandDelay    % Minimum delay between commands (seconds)
        lastCommandTime % Timestamp of last command sent
        commStats       % Communication statistics struct
    end
    
    properties (Constant, Access = private)
        % Joint angle limits [min, max] in degrees per joint
        JOINT_LIMITS = [
            -140, 150;   % J1
            -90,  90;    % J2
            -145, 150;   % J3
            -90,  90;    % J4
            -145, 150;   % J5
            -175, 175;   % J6
        ];

        % Protocol constants
        HEADER1 = 0xFE;
        HEADER2 = 0xFE;
        FOOTER = 0xFA;
        
        % Command codes
        CMD_POWER_ON = 0x10;
        CMD_POWER_OFF = 0x11;
        CMD_CHECK_RUNNING = 0x12;
        CMD_RELEASE_POWER = 0x13;
        CMD_IS_CONTROLLER_CONNECTED = 0x14;
        CMD_SET_FREE_MOVE = 0x1A;
        CMD_IS_FREE_MOVE = 0x1B;
        CMD_GET_ANGLES = 0x20;
        CMD_SEND_ANGLE = 0x21;
        CMD_SEND_ANGLES = 0x22;
        CMD_GET_COORDS = 0x23;
        CMD_SEND_COORD = 0x24;
        CMD_SEND_COORDS = 0x25;
        CMD_PAUSE = 0x26;
        CMD_IS_PAUSED = 0x27;
        CMD_RESUME = 0x28;
        CMD_STOP = 0x29;
        CMD_IS_IN_POSITION = 0x2A;
        CMD_IS_MOVING = 0x2B;
        CMD_JOG_ANGLE = 0x30;
        CMD_JOG_ABSOLUTE = 0x31;
        CMD_JOG_COORD = 0x32;
        CMD_JOG_STOP = 0x34;
        CMD_SET_ENCODER = 0x3A;
        CMD_GET_ENCODER = 0x3B;
        CMD_SET_ENCODERS = 0x3C;
        CMD_GET_ENCODERS = 0x3D;
        CMD_GET_SPEED = 0x40;
        CMD_SET_SPEED = 0x41;
        CMD_GET_JOINT_MIN = 0x4A;
        CMD_GET_JOINT_MAX = 0x4B;
        CMD_SET_JOINT_MIN = 0x4C;
        CMD_SET_JOINT_MAX = 0x4D;
        CMD_IS_SERVO_ENABLED = 0x50;
        CMD_IS_ALL_SERVO_ENABLED = 0x51;
        CMD_SET_SERVO_DATA = 0x52;
        CMD_GET_SERVO_DATA = 0x53;
        CMD_SET_SERVO_CALIBRATION = 0x54;
        CMD_RELEASE_SERVO = 0x56;
        CMD_FOCUS_SERVO = 0x57;
        CMD_SET_PIN_MODE = 0x60;
        CMD_SET_DIGITAL_OUTPUT = 0x61;
        CMD_GET_DIGITAL_INPUT = 0x62;
        CMD_GET_GRIPPER_VALUE = 0x65;
        CMD_SET_GRIPPER_STATE = 0x66;
        CMD_SET_GRIPPER_VALUE = 0x67;
        CMD_SET_GRIPPER_INIT = 0x68;
        CMD_IS_GRIPPER_MOVING = 0x69;
        CMD_SET_COLOR = 0x6A;
        CMD_SET_BASIC_OUTPUT = 0xA0;
        CMD_GET_BASIC_OUTPUT = 0xA1;
        CMD_SET_TOOL_REFERENCE = 0x81;
        CMD_GET_TOOL_REFERENCE = 0x82;
        CMD_SET_WORLD_REFERENCE = 0x83;
        CMD_GET_WORLD_REFERENCE = 0x84;
        CMD_SET_REFERENCE_FRAME = 0x85;
        CMD_GET_REFERENCE_FRAME = 0x86;
        CMD_SET_END_TYPE = 0x89;
        CMD_GET_END_TYPE = 0x8A;
    end
    
    methods
        function obj = MyCobot280(portName, varargin)
            % Constructor - Initialize serial connection to myCobot 280
            %
            % Inputs:
            %   portName - Serial port name (e.g., 'COM3', '/dev/ttyUSB0')
            %   varargin - Optional name-value pairs:
            %              'BaudRate' - Baud rate (default: 115200)
            %              'Timeout' - Timeout in seconds (default: 0.5)
            %              'Debug' - Enable debug mode (default: false)
            
            p = inputParser;
            addRequired(p, 'portName', @ischar);
            addParameter(p, 'BaudRate', 115200, @isnumeric);
            addParameter(p, 'Timeout', 5, @isnumeric);
            addParameter(p, 'Debug', false, @islogical);
            addParameter(p, 'MaxRetries', 3, @isnumeric);
            addParameter(p, 'CommandDelay', 0.05, @isnumeric);
            parse(p, portName, varargin{:});

            obj.portName = p.Results.portName;
            obj.baudRate = p.Results.BaudRate;
            obj.timeout = p.Results.Timeout;
            obj.debugMode = p.Results.Debug;
            obj.maxRetries = p.Results.MaxRetries;
            obj.commandDelay = p.Results.CommandDelay;
            obj.lastCommandTime = 0;
            obj.commStats = struct( ...
                'txCount', 0, ...        % Total commands sent
                'rxCount', 0, ...        % Successful responses received
                'txBytes', 0, ...        % Total bytes transmitted
                'rxBytes', 0, ...        % Total bytes received
                'retries', 0, ...        % Total retry attempts
                'timeouts', 0, ...       % Timeout errors
                'frameErrors', 0, ...    % Header/footer/command mismatch errors
                'lastError', '', ...     % Last error message
                'lastErrorTime', '' ...  % Timestamp of last error
            );

            % Initialize serial port
            try
                obj.serialPort = serialport(obj.portName, obj.baudRate);
                configureTerminator(obj.serialPort, "CR/LF");
                obj.serialPort.Timeout = obj.timeout;
                
                % Clear any pending data
                flush(obj.serialPort);
                
                if obj.debugMode
                    fprintf('Connected to myCobot 280 on %s at %d baud\n', ...
                        obj.portName, obj.baudRate);
                end
                
                % Verify connection by checking robot status
                pause(0.1);  % Small delay for port stabilization
                try
                    % Try to get controller status as a connection test
                    obj.isControllerConnected();
                    if obj.debugMode
                        fprintf('Communication verified successfully\n');
                    end
                catch
                    % If first attempt fails, try once more
                    pause(0.2);
                    try
                        obj.isControllerConnected();
                    catch ME
                        warning('MyCobot280:commCheck', '%s', ME.message);
                        warning('Connection established but robot may not be responding');
                    end
                end
                
            catch ME
                error('Failed to open serial port %s: %s', obj.portName, ME.message);
            end
        end
        
        function delete(obj)
            % Destructor - Clean up serial connection
            if ~isempty(obj.serialPort)
                try
                    flush(obj.serialPort);
                    delete(obj.serialPort);
                    if obj.debugMode
                        fprintf('Disconnected from myCobot 280\n');
                    end
                catch
                    % Silent cleanup
                end
            end
        end
        
        %% Power Control Methods
        
        function powerOn(obj)
            % Power on the robot
            obj.sendCommand(obj.CMD_POWER_ON);
            pause(0.5);  % Give robot time to power up
        end
        
        function powerOff(obj)
            % Power off the robot
            obj.sendCommand(obj.CMD_POWER_OFF);
        end
        
        function releasePower(obj)
            % Release power (power down)
            obj.sendCommand(obj.CMD_RELEASE_POWER);
        end
        
        function status = isPoweredOn(obj)
            % Check if robot is powered on
            response = obj.sendCommandWithResponse(obj.CMD_CHECK_RUNNING, 1);
            status = response(1) == 1;
        end
        
        function connected = isControllerConnected(obj)
            % Check if controller is connected
            response = obj.sendCommandWithResponse(obj.CMD_IS_CONTROLLER_CONNECTED, 1);
            connected = response(1) == 1;
        end
        
        %% Movement Control Methods
        
        function setFreeMove(obj, enable)
            % Enable or disable free move mode (torque off)
            % Inputs:
            %   enable - true to enable free move, false to disable
            data = uint8(enable);
            obj.sendCommand(obj.CMD_SET_FREE_MOVE, data);
        end
        
        function freeMove = isFreeMove(obj)
            % Check if robot is in free move mode
            response = obj.sendCommandWithResponse(obj.CMD_IS_FREE_MOVE, 1);
            freeMove = response(1) == 1;
        end
        
        function angles = getAngles(obj)
            % Get current joint angles
            % Output:
            %   angles - 2x6 vector of joint angles as [HighByte LowByte]
            
            response = obj.sendCommandWithResponse(obj.CMD_GET_ANGLES, 12);
            angles = zeros(1, 6);
            
            for i = 1:6
                highByte = response(2*i - 1);
                lowByte = response(2*i);
                temp = lowByte + highByte * 256;
                
                if temp > 32767
                    temp = temp - 65536;
                end
                angles(i) = temp / 100;
            end
        end
        
        function sendAngle(obj, jointID, angle, speed)
            % Send single joint to specified angle
            % Inputs:
            %   jointID - Joint number (1-6)
            %   angle - Target angle in degrees
            %   speed - Speed (0-100)
            
            if jointID < 1 || jointID > 6
                error('Joint ID must be between 1 and 6');
            end

            obj.validateAngle(jointID, angle);

            if speed < 0 || speed > 100
                error('Speed must be between 0 and 100');
            end

            % Convert angle to protocol format
            angleInt = int16(angle * 100);
            u = typecast(angleInt, 'uint16');
            angleHigh = uint8(bitshift(u, -8));
            angleLow = uint8(bitand(u, uint16(255)));

            data = [uint8(jointID), angleHigh, angleLow, uint8(speed)];
            obj.sendCommand(obj.CMD_SEND_ANGLE, data);
        end
        
        function sendAngles(obj, angles, speed)
            % Send all joints to specified angles
            % Inputs:
            %   angles - 1x6 vector of target angles in degrees
            %   speed - Speed (0-100)
            
            if length(angles) ~= 6
                error('Must provide exactly 6 angles');
            end

            for i = 1:6
                obj.validateAngle(i, angles(i));
            end

            if speed < 0 || speed > 100
                error('Speed must be between 0 and 100');
            end

            data = zeros(1, 13, 'uint8');

            for i = 1:6
                angleInt = int16(angles(i) * 100);        % signed 16-bit value
                u = typecast(angleInt, 'uint16');                     % work in uint16 for bit ops
                data(2*i - 1) = uint8(bitshift(u, -8));   % high byte
                data(2*i)     = uint8(bitand(u, uint16(255))); % low byte
            end
            
            data(13) = uint8(speed);
            obj.sendCommand(obj.CMD_SEND_ANGLES, data);
        end
        
        function coords = getCoords(obj)
            % Get current end effector coordinates
            % Output:
            %   coords - 1x6 vector [x, y, z, rx, ry, rz]
            %            xyz in mm, rx/ry/rz in degrees
            
            response = obj.sendCommandWithResponse(obj.CMD_GET_COORDS, 12);
            coords = zeros(1, 6);
            
            for i = 1:3  % x, y, z
                highByte = response(2*i - 1);
                lowByte = response(2*i);
                temp = lowByte + highByte * 256;

                if temp > 32767
                    temp = temp - 65536;
                end
                coords(i) = temp / 10;  % Convert to mm
            end

            for i = 4:6  % rx, ry, rz
                highByte = response(2*i - 1);
                lowByte = response(2*i);
                temp = lowByte + highByte * 256;

                if temp > 32767
                    temp = temp - 65536;
                end
                coords(i) = temp / 100;  % Convert to degrees
            end
        end

        function sendCoord(obj, axis, value, speed)
            % Send single coordinate axis to specified value
            % Inputs:
            %   axis - Axis number (1=x, 2=y, 3=z, 4=rx, 5=ry, 6=rz)
            %   value - Target value (mm for xyz, degrees for rx/ry/rz)
            %   speed - Speed (0-100)
            
            if axis < 1 || axis > 6
                error('Axis must be between 1 and 6');
            end
            
            if speed < 0 || speed > 100
                error('Speed must be between 0 and 100');
            end
            
            % Convert value based on axis type
            if axis <= 3  % xyz coordinates
                valueInt = int16(value * 10);
            else  % rx, ry, rz angles
                valueInt = int16(value * 100);
            end

            u = typecast(valueInt, 'uint16');
            valueHigh = uint8(bitshift(u, -8));
            valueLow = uint8(bitand(u, uint16(255)));

            data = [uint8(axis), valueHigh, valueLow, uint8(speed)];
            obj.sendCommand(obj.CMD_SEND_COORD, data);
        end
        
        function sendCoords(obj, coords, speed, mode)
            % Send all coordinates
            % Inputs:
            %   coords - 1x6 vector [x, y, z, rx, ry, rz]
            %   speed - Speed (0-100)
            %   mode - Motion mode (default: 1)
            
            if nargin < 4
                mode = 1;
            end
            
            if length(coords) ~= 6
                error('Must provide exactly 6 coordinates');
            end
            
            if speed < 0 || speed > 100
                error('Speed must be between 0 and 100');
            end
            
            data = zeros(1, 14, 'uint8');
            
            % xyz coordinates
            for i = 1:3
                valueInt = int16(coords(i) * 10);
                u = typecast(valueInt, 'uint16');
                data(2*i - 1) = uint8(bitshift(u, -8));
                data(2*i) = uint8(bitand(u, uint16(255)));
            end

            % rx, ry, rz angles
            for i = 4:6
                valueInt = int16(coords(i) * 100);
                u = typecast(valueInt, 'uint16');
                data(2*i - 1) = uint8(bitshift(u, -8));
                data(2*i) = uint8(bitand(u, uint16(255)));
            end

            data(13) = uint8(speed);
            data(14) = uint8(mode);

            obj.sendCommand(obj.CMD_SEND_COORDS, data);
        end
        
        %% Motion Status Methods
        
        function pause(obj)
            % Pause current motion
            obj.sendCommand(obj.CMD_PAUSE);
        end
        
        function resume(obj)
            % Resume paused motion
            obj.sendCommand(obj.CMD_RESUME);
        end
        
        function stop(obj)
            % Stop all motion
            obj.sendCommand(obj.CMD_STOP);
        end
        
        function paused = isPaused(obj)
            % Check if robot motion is paused
            response = obj.sendCommandWithResponse(obj.CMD_IS_PAUSED, 1);
            paused = response(1) == 1;
        end
        
        function moving = isMoving(obj)
            % Check if robot is currently moving
            response = obj.sendCommandWithResponse(obj.CMD_IS_MOVING, 1);
            moving = response(1) == 1;
        end
        
        function inPosition = isInPosition(obj, target, isAngles)
            % Check if robot is at target position
            % Inputs:
            %   target - 1x6 vector of target angles or coordinates
            %   isAngles - true for angles, false for coordinates
            
            if nargin < 3
                isAngles = false;  % Default to coordinates
            end
            
            data = zeros(1, 13, 'uint8');
            
            if isAngles
                % Format angle data
                for i = 1:6
                    angleInt = int16(target(i) * 100);
                    u = typecast(angleInt, 'uint16');
                    data(2*i - 1) = uint8(bitshift(u, -8));
                    data(2*i) = uint8(bitand(u, uint16(255)));
                end
                data(13) = 0;  % Angle mode
            else
                % Format coordinate data
                for i = 1:3
                    valueInt = int16(target(i) * 10);
                    u = typecast(valueInt, 'uint16');
                    data(2*i - 1) = uint8(bitshift(u, -8));
                    data(2*i) = uint8(bitand(u, uint16(255)));
                end
                for i = 4:6
                    valueInt = int16(target(i) * 100);
                    u = typecast(valueInt, 'uint16');
                    data(2*i - 1) = uint8(bitshift(u, -8));
                    data(2*i) = uint8(bitand(u, uint16(255)));
                end
                data(13) = 1;  % Coordinate mode
            end
            
            response = obj.sendCommandWithResponse(obj.CMD_IS_IN_POSITION, 1, data);
            inPosition = response(1) == 1;
        end
        
        %% Jog Control Methods
        
        function jogAngle(obj, jointID, direction, speed)
            % Jog single joint
            % Inputs:
            %   jointID - Joint number (1-6)
            %   direction - 1 for positive, 0 for negative
            %   speed - Speed (0-100)
            
            if jointID < 1 || jointID > 6
                error('Joint ID must be between 1 and 6');
            end
            
            if speed < 0 || speed > 100
                error('Speed must be between 0 and 100');
            end
            
            data = [uint8(jointID), uint8(direction), uint8(speed)];
            obj.sendCommand(obj.CMD_JOG_ANGLE, data);
        end
        
        function jogCoord(obj, axis, direction, speed)
            % Jog along coordinate axis
            % Inputs:
            %   axis - Axis number (1=x, 2=y, 3=z, 4=rx, 5=ry, 6=rz)
            %   direction - 1 for positive, 0 for negative
            %   speed - Speed (0-100)
            
            if axis < 1 || axis > 6
                error('Axis must be between 1 and 6');
            end
            
            if speed < 0 || speed > 100
                error('Speed must be between 0 and 100');
            end
            
            data = [uint8(axis), uint8(direction), uint8(speed)];
            obj.sendCommand(obj.CMD_JOG_COORD, data);
        end
        
        function jogStop(obj)
            % Stop jogging motion
            obj.sendCommand(obj.CMD_JOG_STOP);
        end
        
        %% Speed Control Methods
        
        function speed = getSpeed(obj)
            % Get current speed setting
            response = obj.sendCommandWithResponse(obj.CMD_GET_SPEED, 1);
            speed = double(response(1));
        end
        
        function setSpeed(obj, speed)
            % Set global speed
            % Input:
            %   speed - Speed value (0-100)
            
            if speed < 0 || speed > 100
                error('Speed must be between 0 and 100');
            end
            
            data = uint8(speed);
            obj.sendCommand(obj.CMD_SET_SPEED, data);
        end
        
        %% Gripper Control Methods
        
        function value = getGripperValue(obj)
            % Get current gripper opening value (0-100)
            response = obj.sendCommandWithResponse(obj.CMD_GET_GRIPPER_VALUE, 1);
            value = double(response(1));
        end
        
        function setGripperState(obj, state, speed)
            % Set gripper state
            % Inputs:
            %   state - 0 for open, 1 for close
            %   speed - Speed (0-100)
            
            if speed < 0 || speed > 100
                error('Speed must be between 0 and 100');
            end
            
            data = [uint8(state), uint8(speed)];
            obj.sendCommand(obj.CMD_SET_GRIPPER_STATE, data);
        end
        
        function setGripperValue(obj, value, speed)
            % Set gripper to specific opening value
            % Inputs:
            %   value - Opening value (0-100)
            %   speed - Speed (0-100)
            
            if value < 0 || value > 100
                error('Value must be between 0 and 100');
            end
            
            if speed < 0 || speed > 100
                error('Speed must be between 0 and 100');
            end
            
            data = [uint8(value), uint8(speed)];
            obj.sendCommand(obj.CMD_SET_GRIPPER_VALUE, data);
        end
        
        function setGripperInit(obj)
            % Initialize gripper to zero position
            obj.sendCommand(obj.CMD_SET_GRIPPER_INIT);
        end
        
        function moving = isGripperMoving(obj)
            % Check if gripper is moving
            response = obj.sendCommandWithResponse(obj.CMD_IS_GRIPPER_MOVING, 1);
            moving = response(1) == 1;
        end
        
        %% LED Control Methods
        
        function setColor(obj, r, g, b)
            % Set RGB color of Atom LED
            % Inputs:
            %   r - Red value (0-255)
            %   g - Green value (0-255)
            %   b - Blue value (0-255)
            
            if r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255
                error('RGB values must be between 0 and 255');
            end
            
            data = [uint8(r), uint8(g), uint8(b)];
            obj.sendCommand(obj.CMD_SET_COLOR, data);
        end
        
        %% IO Control Methods
        
        function setPinMode(obj, pinNo, mode)
            % Set pin mode for Atom
            % Inputs:
            %   pinNo - Pin number
            %   mode - 0 for input, 1 for output
            
            data = [uint8(pinNo), uint8(mode)];
            obj.sendCommand(obj.CMD_SET_PIN_MODE, data);
        end
        
        function setDigitalOutput(obj, pinNo, level)
            % Set digital output on Atom pin
            % Inputs:
            %   pinNo - Pin number
            %   level - 0 for low, 1 for high
            
            data = [uint8(pinNo), uint8(level)];
            obj.sendCommand(obj.CMD_SET_DIGITAL_OUTPUT, data);
        end
        
        function [pinNo, level] = getDigitalInput(obj, pinNo)
            % Get digital input from Atom pin
            % Input:
            %   pinNo - Pin number
            % Outputs:
            %   pinNo - Pin number (echo)
            %   level - Pin level (0 or 1)
            
            data = uint8(pinNo);
            response = obj.sendCommandWithResponse(obj.CMD_GET_DIGITAL_INPUT, 2, data);
            pinNo = double(response(1));
            level = double(response(2));
        end
        
        function setBasicOutput(obj, pinNo, level)
            % Set basic IO output
            % Inputs:
            %   pinNo - Pin number
            %   level - 0 for low, 1 for high
            
            data = [uint8(pinNo), uint8(level)];
            obj.sendCommand(obj.CMD_SET_BASIC_OUTPUT, data);
        end
        
        function [pinNo, level] = getBasicOutput(obj, pinNo)
            % Get basic IO output state
            % Input:
            %   pinNo - Pin number
            % Outputs:
            %   pinNo - Pin number (echo)
            %   level - Pin level (0 or 1)
            
            data = uint8(pinNo);
            response = obj.sendCommandWithResponse(obj.CMD_GET_BASIC_OUTPUT, 2, data);
            pinNo = double(response(1));
            level = double(response(2));
        end
        
        %% Coordinate System Methods
        
        function setToolReference(obj, coords)
            % Set tool coordinate system
            % Input:
            %   coords - 1x6 vector [x, y, z, rx, ry, rz]
            
            if length(coords) ~= 6
                error('Must provide exactly 6 coordinates');
            end
            
            data = zeros(1, 12, 'uint8');

            % xyz coordinates
            for i = 1:3
                valueInt = int16(coords(i) * 10);
                u = typecast(valueInt, 'uint16');
                data(2*i - 1) = uint8(bitshift(u, -8));
                data(2*i) = uint8(bitand(u, uint16(255)));
            end

            % rx, ry, rz angles
            for i = 4:6
                valueInt = int16(coords(i) * 100);
                u = typecast(valueInt, 'uint16');
                data(2*i - 1) = uint8(bitshift(u, -8));
                data(2*i) = uint8(bitand(u, uint16(255)));
            end

            obj.sendCommand(obj.CMD_SET_TOOL_REFERENCE, data);
        end
        
        function coords = getToolReference(obj)
            % Get tool coordinate system
            % Output:
            %   coords - 1x6 vector [x, y, z, rx, ry, rz]
            
            response = obj.sendCommandWithResponse(obj.CMD_GET_TOOL_REFERENCE, 12);
            coords = zeros(1, 6);
            
            for i = 1:3  % x, y, z
                highByte = response(2*i - 1);
                lowByte = response(2*i);
                temp = lowByte + highByte * 256;

                if temp > 32767
                    temp = temp - 65536;
                end
                coords(i) = temp / 10;  % Convert to mm
            end

            for i = 4:6  % rx, ry, rz
                highByte = response(2*i - 1);
                lowByte = response(2*i);
                temp = lowByte + highByte * 256;

                if temp > 32767
                    temp = temp - 65536;
                end
                coords(i) = temp / 100;  % Convert to degrees
            end
        end

        function setReferenceFrame(obj, rfType)
            % Set reference frame type
            % Input:
            %   rfType - 0 for base, 1 for world
            
            data = uint8(rfType);
            obj.sendCommand(obj.CMD_SET_REFERENCE_FRAME, data);
        end
        
        function rfType = getReferenceFrame(obj)
            % Get reference frame type
            % Output:
            %   rfType - 0 for base, 1 for world
            
            response = obj.sendCommandWithResponse(obj.CMD_GET_REFERENCE_FRAME, 1);
            rfType = double(response(1));
        end
        
        function setEndType(obj, endType)
            % Set end effector type
            % Input:
            %   endType - 0 for flange, 1 for tool
            
            data = uint8(endType);
            obj.sendCommand(obj.CMD_SET_END_TYPE, data);
        end
        
        function endType = getEndType(obj)
            % Get end effector type
            % Output:
            %   endType - 0 for flange, 1 for tool
            
            response = obj.sendCommandWithResponse(obj.CMD_GET_END_TYPE, 1);
            endType = double(response(1));
        end
        
        %% Additional Utility Methods
        
        function ready = isReady(obj)
            % Check if robot is ready for commands
            % Output:
            %   ready - true if robot is responding properly
            
            try
                % Try to check controller connection
                obj.isControllerConnected();
                ready = true;
            catch
                ready = false;
            end
        end
        
        function clearCommunication(obj)
            % Clear communication buffers and reset connection
            
            if obj.debugMode
                fprintf('Clearing communication buffers...\n');
            end
            
            % Flush both input and output buffers
            flush(obj.serialPort);
            
            % Small delay
            pause(0.1);
            
            % Clear any remaining bytes
            if obj.serialPort.NumBytesAvailable > 0
                discarded = read(obj.serialPort, obj.serialPort.NumBytesAvailable, 'uint8');
                if obj.debugMode
                    fprintf('Discarded %d bytes from buffer\n', length(discarded));
                end
            end
        end
        
        function waitForIdle(obj, timeout)
            % Wait for robot to stop moving
            % Input:
            %   timeout - Maximum wait time in seconds (default: 10)
            
            if nargin < 2
                timeout = 10;
            end
            
            startTime = tic;
            while obj.isMoving()
                if toc(startTime) > timeout
                    warning('Timeout waiting for robot to stop');
                    break;
                end
                pause(0.05);
            end
        end
        
        function validateAngle(obj, jointID, angle)
            % Validate that angle is within the allowed range for a joint
            minAngle = obj.JOINT_LIMITS(jointID, 1);
            maxAngle = obj.JOINT_LIMITS(jointID, 2);
            if angle < minAngle || angle > maxAngle
                error('Joint %d angle %.1f is out of range [%d, %d] degrees', ...
                    jointID, angle, minAngle, maxAngle);
            end
        end

        function bytesAvailable = getBytesAvailable(obj)
            % Get number of bytes available in serial buffer
            % Output:
            %   bytesAvailable - Number of bytes waiting to be read

            bytesAvailable = obj.serialPort.NumBytesAvailable;
        end

        %% Communication Diagnostics

        function stats = getCommStats(obj)
            % Get communication statistics
            % Output:
            %   stats - Struct with TX/RX counts, bytes, errors, etc.
            %
            % Example:
            %   s = myc.getCommStats();
            %   fprintf('Success rate: %.1f%%\n', s.successRate);

            stats = obj.commStats;

            % Compute derived metrics
            if stats.txCount > 0
                stats.successRate = 100 * stats.rxCount / stats.txCount;
            else
                stats.successRate = 0;
            end
        end

        function resetCommStats(obj)
            % Reset all communication statistics to zero

            obj.commStats.txCount = 0;
            obj.commStats.rxCount = 0;
            obj.commStats.txBytes = 0;
            obj.commStats.rxBytes = 0;
            obj.commStats.retries = 0;
            obj.commStats.timeouts = 0;
            obj.commStats.frameErrors = 0;
            obj.commStats.lastError = '';
            obj.commStats.lastErrorTime = '';
        end

        function printCommStats(obj)
            % Print communication statistics to the console

            s = obj.getCommStats();
            fprintf('\n--- MyCobot280 Communication Stats ---\n');
            fprintf('  TX:  %d commands (%d bytes)\n', s.txCount, s.txBytes);
            fprintf('  RX:  %d responses (%d bytes)\n', s.rxCount, s.rxBytes);
            fprintf('  Success rate:  %.1f%%\n', s.successRate);
            fprintf('  Retries:       %d\n', s.retries);
            fprintf('  Timeouts:      %d\n', s.timeouts);
            fprintf('  Frame errors:  %d\n', s.frameErrors);
            if ~isempty(s.lastError)
                fprintf('  Last error:    %s\n', s.lastError);
                fprintf('  Error time:    %s\n', s.lastErrorTime);
            end
            fprintf('--------------------------------------\n');
        end

        function report = checkConnection(obj)
            % Run a diagnostic check on the serial connection
            % Output:
            %   report - Struct with connection health details
            %
            % Example:
            %   r = myc.checkConnection();
            %   disp(r.summary);

            report = struct( ...
                'portOpen', false, ...
                'portName', obj.portName, ...
                'controllerResponds', false, ...
                'powerOn', false, ...
                'anglesReadable', false, ...
                'roundTripMs', NaN, ...
                'summary', '' ...
            );

            % Check port is open
            try
                report.portOpen = (obj.serialPort.NumBytesAvailable >= 0);
            catch
                report.summary = 'FAIL: Serial port is not open';
                fprintf('%s\n', report.summary);
                return;
            end

            % Check controller responds
            try
                t0 = tic;
                report.controllerResponds = obj.isControllerConnected();
                report.roundTripMs = toc(t0) * 1000;
            catch
                report.summary = 'FAIL: Controller not responding';
                fprintf('%s\n', report.summary);
                return;
            end

            % Check power
            try
                report.powerOn = obj.isPoweredOn();
            catch
                report.powerOn = false;
            end

            % Check angle reading
            try
                angles = obj.getAngles();
                report.anglesReadable = (length(angles) == 6);
            catch
                report.anglesReadable = false;
            end

            % Build summary
            if report.controllerResponds && report.powerOn && report.anglesReadable
                report.summary = sprintf('OK: Connected on %s, round-trip %.0fms', ...
                    obj.portName, report.roundTripMs);
            elseif report.controllerResponds && ~report.powerOn
                report.summary = sprintf('WARN: Connected but robot is powered off (round-trip %.0fms)', ...
                    report.roundTripMs);
            else
                report.summary = 'DEGRADED: Partial communication failure';
            end
            fprintf('%s\n', report.summary);
        end

        function setMotionMode(obj, mode)
            % Set motion mode (interpolation or motion)
            % Input:
            %   mode - 0 for interpolation mode, 1 for motion mode
            % Note: Motion mode is typically needed for movement commands
            
            data = uint8(mode);
            obj.sendCommand(0x16, data);  % Command 0x16 for mode setting
            pause(0.5);  % Allow mode change to take effect
        end

        %% Servo Control Methods
        
        function enabled = isServoEnabled(obj, jointID)
            % Check if specific servo is enabled/connected
            % Input:
            %   jointID - Joint number (1-6)
            % Output:
            %   enabled - true if servo is enabled
            
            if jointID < 1 || jointID > 6
                error('Joint ID must be between 1 and 6');
            end
            
            data = uint8(jointID);
            response = obj.sendCommandWithResponse(obj.CMD_IS_SERVO_ENABLED, 2, data);
            enabled = response(2) == 1;
        end
        
        function allEnabled = isAllServoEnabled(obj)
            % Check if all servos are enabled
            % Output:
            %   allEnabled - true if all servos are enabled
            
            response = obj.sendCommandWithResponse(obj.CMD_IS_ALL_SERVO_ENABLED, 1);
            allEnabled = response(1) == 1;
        end
        
        function setServoData(obj, jointID, dataID, value)
            % Set servo parameter
            % Inputs:
            %   jointID - Joint number (1-6)
            %   dataID - Parameter address (see documentation)
            %   value - Parameter value
            
            if jointID < 1 || jointID > 6
                error('Joint ID must be between 1 and 6');
            end
            
            data = [uint8(jointID), uint8(dataID), uint8(value)];
            obj.sendCommand(obj.CMD_SET_SERVO_DATA, data);
        end
        
        function value = getServoData(obj, jointID, dataID)
            % Get servo parameter
            % Inputs:
            %   jointID - Joint number (1-6)
            %   dataID - Parameter address (see documentation)
            % Output:
            %   value - Parameter value
            
            if jointID < 1 || jointID > 6
                error('Joint ID must be between 1 and 6');
            end
            
            data = [uint8(jointID), uint8(dataID)];
            response = obj.sendCommandWithResponse(obj.CMD_GET_SERVO_DATA, 1, data);
            value = double(response(1));
        end
        
        function focusServo(obj, jointID)
            % Focus (power on) a specific servo motor
            % Input:
            %   jointID - Joint number (1-6)
            
            if jointID < 1 || jointID > 6
                error('Joint ID must be between 1 and 6');
            end
            
            data = uint8(jointID);
            obj.sendCommand(obj.CMD_FOCUS_SERVO, data);
        end
        
        function releaseServo(obj, jointID)
            % Release (power off) a specific servo motor
            % Input:
            %   jointID - Joint number (1-6)
            
            if jointID < 1 || jointID > 6
                error('Joint ID must be between 1 and 6');
            end
            
            data = uint8(jointID);
            obj.sendCommand(obj.CMD_RELEASE_SERVO, data);
        end
        
    end
    
    methods (Access = public)
        function sendCommand(obj, command, data)
            % Send command to robot (fire-and-forget, no response expected)
            % Inputs:
            %   command - Command byte
            %   data - Optional data bytes (uint8 array)

            if nargin < 3
                data = [];
            end

            % Enforce minimum delay between commands
            obj.waitForCommandDelay();

            % Build frame: [FE FE LEN CMD DATA... FA]
            dataLength = 2 + length(data);
            message = [obj.HEADER1, obj.HEADER2, uint8(dataLength), uint8(command)];
            if ~isempty(data)
                message = [message, data];
            end
            message = [message, obj.FOOTER];

            % Flush stale input before sending
            if obj.serialPort.NumBytesAvailable > 0
                if obj.debugMode
                    fprintf('Cleared %d stale bytes before send\n', ...
                        obj.serialPort.NumBytesAvailable);
                end
                flush(obj.serialPort, 'input');
            end

            % Send message
            write(obj.serialPort, message, 'uint8');
            obj.lastCommandTime = toc(uint64(0));

            % Update TX stats
            obj.commStats.txCount = obj.commStats.txCount + 1;
            obj.commStats.txBytes = obj.commStats.txBytes + length(message);

            if obj.debugMode
                fprintf('TX [#%d]: ', obj.commStats.txCount);
                fprintf('%02X ', message);
                fprintf('\n');
            end
        end

        function response = sendCommandWithResponse(obj, command, expectedBytes, data)
            % Send command and wait for response, with retry logic.
            % Retries up to maxRetries times on timeout or frame errors,
            % matching pymycobot's _res() retry pattern.
            %
            % Inputs:
            %   command - Command byte
            %   expectedBytes - Number of expected data bytes in response
            %   data - Optional data bytes to send (uint8 array)
            % Output:
            %   response - Response data bytes (without headers/footer)

            if nargin < 4
                data = [];
            end

            lastErr = [];
            for attempt = 1:obj.maxRetries
                try
                    % Flush input buffer to discard any stale data
                    flush(obj.serialPort, 'input');

                    % Send command
                    obj.sendCommand(command, data);

                    % Read response frame, scanning for valid header
                    response = obj.readResponseFrame(command, expectedBytes);

                    % Update RX stats on success
                    obj.commStats.rxCount = obj.commStats.rxCount + 1;
                    return;
                catch ME
                    lastErr = ME;
                    obj.recordError(ME, command);

                    % Track whether it was a timeout or frame error
                    if contains(ME.message, 'Timeout')
                        obj.commStats.timeouts = obj.commStats.timeouts + 1;
                    else
                        obj.commStats.frameErrors = obj.commStats.frameErrors + 1;
                    end

                    if attempt < obj.maxRetries
                        obj.commStats.retries = obj.commStats.retries + 1;
                        if obj.debugMode
                            fprintf('Retry %d/%d for cmd 0x%02X: %s\n', ...
                                attempt, obj.maxRetries, command, ME.message);
                        end
                    end

                    % Flush everything and pause before retry
                    flush(obj.serialPort);
                    pause(0.1);
                end
            end
            % All retries exhausted
            error('Command 0x%02X failed after %d attempts. Last error: %s', ...
                command, obj.maxRetries, lastErr.message);
        end

        function response = readResponseFrame(obj, command, ~)
            % Read a response frame, scanning byte-by-byte for a valid
            % FE FE header to resynchronize if the stream is misaligned.
            %
            % Inputs:
            %   command       - Expected command byte in response
            %   ~             - (unused) kept for call-site compatibility
            % Output:
            %   response      - Data bytes (without header/footer)

            maxWaitTime = obj.timeout;
            startTime = tic;

            % Scan for header: two consecutive 0xFE bytes
            syncCount = 0;
            while toc(startTime) < maxWaitTime
                if obj.serialPort.NumBytesAvailable > 0
                    b = read(obj.serialPort, 1, 'uint8');
                    if b == obj.HEADER1
                        syncCount = syncCount + 1;
                        if syncCount >= 2
                            break;  % Found FE FE
                        end
                    else
                        syncCount = 0;  % Reset on non-FE byte
                    end
                else
                    pause(0.001);
                end
            end

            if syncCount < 2
                error('Timeout waiting for response header after %.2f seconds', ...
                    toc(startTime));
            end

            % Read length and command bytes
            lenAndCmd = obj.waitAndRead(2, 'length+command', maxWaitTime - toc(startTime));
            respLen = double(lenAndCmd(1));
            respCmd = lenAndCmd(2);

            if obj.debugMode
                fprintf('RX header: FE FE %02X %02X\n', respLen, respCmd);
            end

            % Verify command matches
            if respCmd ~= command
                error('Response command mismatch. Expected 0x%02X, got 0x%02X', ...
                    command, respCmd);
            end

            % Read remaining payload: respLen includes cmd byte + data + footer,
            % we already read the cmd byte, so remaining = respLen - 1
            remaining = respLen - 1;
            if remaining < 1
                error('Invalid response length: %d', respLen);
            end

            payload = obj.waitAndRead(remaining, 'response payload', ...
                maxWaitTime - toc(startTime));

            % Track RX bytes (header + len + cmd + payload)
            obj.commStats.rxBytes = obj.commStats.rxBytes + 2 + 2 + remaining;

            % Last byte should be footer
            footer = payload(end);
            if footer ~= obj.FOOTER
                error('Invalid response footer: 0x%02X', footer);
            end

            % Data is everything except the footer
            response = payload(1:end-1);

            if obj.debugMode
                fprintf('RX data: ');
                fprintf('%02X ', response);
                fprintf(' | footer: %02X\n', footer);
            end
        end

        function data = waitAndRead(obj, numBytes, description, remainingTimeout)
            % Wait for serial data to be available and read specified bytes
            % Inputs:
            %   numBytes         - Number of bytes to read
            %   description      - Description of data being read (for errors)
            %   remainingTimeout - Optional remaining timeout (seconds)
            % Output:
            %   data - Read bytes

            if nargin < 3
                description = 'data';
            end
            if nargin < 4
                remainingTimeout = obj.timeout;
            end

            maxWaitTime = max(remainingTimeout, 0.1);
            pollInterval = 0.001;
            startTime = tic;

            while obj.serialPort.NumBytesAvailable < numBytes
                if toc(startTime) > maxWaitTime
                    availableBytes = obj.serialPort.NumBytesAvailable;
                    error('Timeout waiting for %s. Expected %d bytes, got %d bytes after %.2f seconds', ...
                        description, numBytes, availableBytes, maxWaitTime);
                end
                pause(pollInterval);
            end

            data = read(obj.serialPort, numBytes, 'uint8');

            if obj.debugMode
                fprintf('Read %d bytes for %s after %.3f seconds\n', ...
                    numBytes, description, toc(startTime));
            end
        end

        function waitForCommandDelay(obj)
            % Enforce minimum inter-command delay to avoid overwhelming
            % the robot's serial buffer (matches pymycobot's 0.05s pacing)
            if obj.lastCommandTime > 0
                elapsed = toc(uint64(0)) - obj.lastCommandTime;
                if elapsed < obj.commandDelay
                    pause(obj.commandDelay - elapsed);
                end
            end
        end

        function recordError(obj, ME, command)
            % Record an error in commStats
            obj.commStats.lastError = sprintf('cmd 0x%02X: %s', command, ME.message);
            obj.commStats.lastErrorTime = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss.SSS'));
        end
    end
end