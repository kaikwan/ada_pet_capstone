const openGripper = () => executeFollowJointTrajectory(['gripper_aperture'], [0.1]);
const closeGripper = () => executeFollowJointTrajectory(['gripper_aperture'], [-0.03]);
const moveLiftToTop = () => executeFollowJointTrajectory(['joint_lift'], [1.1]);
const moveLiftToMiddle = () => executeFollowJointTrajectory(['joint_lift'], [0.6]);
const extendArmFully = () => executeFollowJointTrajectory(['wrist_extension'], [0.5]);
const retractArmFully = () => executeFollowJointTrajectory(['wrist_extension'], [0.0]);
const resetWristToDefault = () => executeFollowJointTrajectory(['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll'], [0.0, 0.0, 0.0]);

const updateJointSpeed = (speed) => {
    jointIncrement = CLICK_CLICK_SPEEDS[speed];
};

// Function to call the "Move to Home" service
const homeTheRobot = () => {
    const homeService = new ROSLIB.Service({
        ros: ros,
        name: '/home_the_robot',
        serviceType: 'std_srvs/srv/Trigger',
    });

    const request = new ROSLIB.ServiceRequest({});

    homeService.callService(request, (result) => {
        if (result.success) {
            console.log('Robot successfully homed:', result.message);
        } else {
            console.error('Failed to home:', result.message);
        }
    });
};

// Function to call the "Stow the Robot" service
const stowTheRobot = () => {
    const stowService = new ROSLIB.Service({
        ros: ros,
        name: '/stow_the_robot',
        serviceType: 'std_srvs/srv/Trigger',
    });

    const request = new ROSLIB.ServiceRequest({});

    stowService.callService(request, (result) => {
        if (result.success) {
            console.log('Robot successfully stowed:', result.message);
        } else {
            console.error('Failed to stow the robot:', result.message);
        }
    });
};
// Generic function to move the selected joint incrementally
const moveSelectedJoint = (direction) => {
    const newPosition = currentJointPositions[selectedJoint] + direction * jointIncrement;

    // Ensure the new position is within joint limits
    const limits = JOINT_LIMITS[selectedJoint];
    if (newPosition >= limits.min && newPosition <= limits.max) {
        executeFollowJointTrajectory([`joint_wrist_${selectedJoint}`], [newPosition]);
        currentJointPositions[selectedJoint] = newPosition; // Update the current position
    }
};

// Move the selected control (lift or arm) to an absolute position
const moveSelectedControlToAbsolute = (position) => {
    const controlNames = {
        lift: 'joint_lift',
        arm: 'wrist_extension',
    };

    executeFollowJointTrajectory([controlNames[selectedControl]], [position]);
    currentControlPositions[selectedControl] = position; // Update the current position
};

// Generic function to move the selected control incrementally
const moveSelectedControl = (direction) => {
    const increment = jointIncrement; // Use the same joint increment
    const newPosition = currentControlPositions[selectedControl] + direction * increment;

    // Ensure the new position is within limits
    const limits = LIFT_ARM_LIMITS[selectedControl];
    if (newPosition >= limits.min && newPosition <= limits.max) {
        moveSelectedControlToAbsolute(newPosition);
    }
};

const moveSelectedJointToAbsolute = (position) => {
    const jointNames = {
        yaw: 'joint_wrist_yaw',
        pitch: 'joint_wrist_pitch',
        roll: 'joint_wrist_roll',
    };

    executeFollowJointTrajectory([jointNames[selectedJoint]], [position]);
};
// Function to toggle the runstop state
const toggleRunstop = () => {
    runstopState = !runstopState; // Toggle the state

    const runstopService = new ROSLIB.Service({
        ros: ros,
        name: '/runstop',
        serviceType: 'std_srvs/srv/SetBool',
    });

    const request = new ROSLIB.ServiceRequest({
        data: runstopState,
    });

    runstopService.callService(request, (result) => {
        if (result.success) {
            console.log(`Runstop state set to ${runstopState}:`, result.message);
            updateRunstopButton();
        } else {
            console.error('Failed to toggle runstop:', result.message);
        }
    });
};
