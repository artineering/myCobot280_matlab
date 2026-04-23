%% test_joint_limits.m - Validate joint angle limits for MyCobot 280 M5
% Tests each joint at key angles within and beyond the allowed range.
% Requires the robot to be connected on COM13.

%% Setup
myc = MyCobot280('COM13', 'BaudRate', 115200);
myc.powerOn();
pause(2);

% Go home
fprintf('Going home...\n');
myc.sendAngles([0 0 0 0 0 0], 20);
pause(4);

%% Test each joint
limits = [
    -140, 150;   % J1
     -90,  90;   % J2
    -145, 150;   % J3
     -90,  90;   % J4
    -145, 150;   % J5
    -175, 175;   % J6
];

testAngles = [-140 -90 -45 0 45 90 140];
speed = 25;
tolerance = 3;  % degrees

results = {};
nPass = 0;
nFail = 0;

for joint = 1:6
    fprintf('\n========== Joint %d (limits: [%d, %d]) ==========\n', ...
        joint, limits(joint,1), limits(joint,2));

    for idx = 1:length(testAngles)
        angle = testAngles(idx);

        % Skip angles outside this joint's limits
        if angle < limits(joint,1) || angle > limits(joint,2)
            fprintf('  J%d -> %+4d : SKIP (outside limit)\n', joint, angle);
            continue;
        end

        % Send joint to angle
        myc.sendAngle(joint, angle, speed);
        pause(4);

        % Read back
        actual = myc.getAngles();
        measured = actual(joint);
        err = abs(measured - angle);

        if err <= tolerance
            status = 'PASS';
            nPass = nPass + 1;
        else
            status = 'FAIL';
            nFail = nFail + 1;
        end

        fprintf('  J%d -> %+4d : actual=%+7.2f  err=%.1f  [%s]\n', ...
            joint, angle, measured, err, status);

        results{end+1} = struct('joint', joint, 'commanded', angle, ...
            'actual', measured, 'error', err, 'status', status); %#ok<SAGROW>
    end

    % Return joint to zero before moving to next
    myc.sendAngle(joint, 0, speed);
    pause(3);
end

%% Test that out-of-range angles are rejected
fprintf('\n========== Out-of-range rejection tests ==========\n');
outOfRange = [
    1, -145;
    1,  155;
    2, -100;
    2,  100;
    4, -100;
    4,  100;
    6, -180;
    6,  180;
];

for k = 1:size(outOfRange, 1)
    joint = outOfRange(k, 1);
    angle = outOfRange(k, 2);
    try
        myc.sendAngle(joint, angle, 20);
        fprintf('  J%d -> %+4d : FAIL (should have been rejected)\n', joint, angle);
        nFail = nFail + 1;
        % Safety: return to zero if it somehow went through
        myc.sendAngle(joint, 0, 20);
        pause(3);
    catch ME
        fprintf('  J%d -> %+4d : PASS (rejected: %s)\n', joint, angle, ME.message);
        nPass = nPass + 1;
    end
end

%% Summary
fprintf('\n========== Summary ==========\n');
fprintf('Total: %d  |  Pass: %d  |  Fail: %d\n', nPass + nFail, nPass, nFail);

% Return home and clean up
fprintf('Returning home...\n');
myc.sendAngles([0 0 0 0 0 0], 20);
pause(4);
delete(myc); clear myc;
fprintf('Done.\n');
