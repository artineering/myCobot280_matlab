classdef MyCobot280
    %MYCOBOT280 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        port
        baudRate
    end

    properties(Constant, Access = private)
        FRAME_START_BYTE = 0xFE
        FRAME_END_BYTE = OxFA
        POWER_UP_CMD = 0x10
        POWER_DOWN_CMD = 0x11
        CHECK_STATUS_CMD = 0x12
        CHECK_SYSTEM_CMD = 0x14
        MODE_UPDATE_CMD = 0x16
        FREE_MODE_CMD = 0x1A
        IS_FREE_MODE_CMD = 0x1B
        BLOCKING_READ_ANGLES_CMD = 0x20
        SET_ANGLE_CMD = 0x21
        SET_ALL_ANGLES_CMD = 0x22
        GET_ALL_COORDINATES_CMD = 0x23
        SET_COORDINATE_CMD = 0x24
        SET_ALL_COORDINATES_CMD = 0x25
        PROGRAM_CMD = 0x27
        STOP_PROGRAM_CMD = 0x29
        COORDINATE_REACHED_CMD = 0x2A
        MOVE_CHECK_CMD = 0x2B
        JOG_JOINT_MOVEMENT_CMD = 0x30
        JOG_ABSOLUTE_CMD = 0x31
        JOG_COORDINATE_MOVEMENT_CMD = 0x32
        JOG_STEPPER_CMD = 0x31
        JOG_STOP_CMD = 0x34
        SET_POTENTIAL_CMD = 0x3A
        GET_POTENTIAL_CMD = 0x3B
        
    end

    methods
        function obj = MyCobot280(port, baudRate)
            obj.port = port;
            obj.baudRate = baudRate;
            obj.serialPortObj = serialport(obj.port, obj.baudRate);
        end


    end
end