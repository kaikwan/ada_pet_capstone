let ros = new ROSLIB.Ros({
    url: 'ws://uw-481-hello-robot-rosbridge-server.loca.lt',
});

let trajectoryClient = null;

// Create subscription to the camera video topic
const subscribeToCameraVideo = () => {
    let cameraImage = document.getElementById("cameraImage");
    let topic = new ROSLIB.Topic({
        ros: ros,
        name: "/camera/color/image_raw/compressed",
        messageType: "sensor_msgs/CompressedImage",
    });
    topic.subscribe((message) => {
        cameraImage.src = "data:image/jpg;base64," + message.data;
    });
};

const subscribeToGripperVideo = () => {
    let gripperImage = document.getElementById("gripperImage");
    let topic = new ROSLIB.Topic({
        ros: ros,
        name: "/gripper_camera/image_raw/compressed",
        messageType: "sensor_msgs/CompressedImage",
    });
    topic.subscribe((message) => {
        gripperImage.src = "data:image/jpg;base64," + message.data;
    });
};

// Create a handle to the FollowJointTrajectory action
const createTrajectoryClient = () => {
    trajectoryClient = new ROSLIB.ActionHandle({
        ros: ros,
        name: "/stretch_controller/follow_joint_trajectory",
        actionType: "control_msgs/action/FollowJointTrajectory",
    });
};

// Execute a FollowJointTrajectory action for given joints
// See valid joints here: https://github.com/hello-robot/stretch_web_teleop/blob/master/src/shared/util.tsx#L4-L20
// and joint limits here: https://github.com/hello-robot/stretch_web_teleop/blob/master/src/shared/util.tsx#L304
const executeFollowJointTrajectory = (jointNames, jointPositions) => {
    let goal = new ROSLIB.ActionGoal({
        trajectory: {
            header: { stamp: { secs: 0, nsecs: 0 } },
            joint_names: jointNames,
            points: [
                {
                    positions: jointPositions,
                    time_from_start: { secs: 1, nsecs: 0 },
                },
            ],
        },
    });
    trajectoryClient.createClient(goal);
};

// Subscribe to the joint states topic to get the current positions of yaw, pitch, roll, lift, and arm
const subscribeToJointStates = () => {
    const jointStateTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/stretch/joint_states',
        messageType: 'sensor_msgs/JointState',
    });

    jointStateTopic.subscribe((message) => {
        const jointIndices = {
            yaw: message.name.indexOf('joint_wrist_yaw'),
            pitch: message.name.indexOf('joint_wrist_pitch'),
            roll: message.name.indexOf('joint_wrist_roll'),
            lift: message.name.indexOf('joint_lift'),
            arm: message.name.indexOf('wrist_extension'),
        };

        for (const joint in jointIndices) {
            if (jointIndices[joint] !== -1) {
                if (joint === 'lift' || joint === 'arm') {
                    currentControlPositions[joint] = message.position[jointIndices[joint]];
                } else {
                    currentJointPositions[joint] = message.position[jointIndices[joint]];
                }
            }
        }

        // Update the wrist slider only if it is not being actively controlled
        if (!isSliderActive) {
            updateSliderForSelectedJoint();
        }

        // Update the control slider only if it is not being actively controlled
        if (!isSliderActive) {
            updateSliderForSelectedControl();
        }
    });
};

let runstopState = false;
let selectedJoint = 'yaw';
let isSliderActive = false;
let currentJointPositions = { yaw: 0.0, pitch: 0.0, roll: 0.0 };
let selectedControl = 'lift';
let currentControlPositions = { lift: 0.0, arm: 0.0 };
let jointInterval = null;
let jointIncrement = CLICK_CLICK_SPEEDS.slow;
let joystickInstance = null;

// Called when the rosbridge websocket connection is successful
ros.on('connection', function () {
    document.getElementById('connection').innerHTML = "Connected to Stretch.";
    document.getElementById('camera').style.display = "block";
    document.getElementById('buttons').style.display = "block";
    console.log('Connected to websocket server.');

    subscribeToCameraVideo();
    subscribeToGripperVideo();
    subscribeToJointStates();
    initializeJoystick();
    createTrajectoryClient();
});
ros.on('error', (error) => {
    document.getElementById('connection').innerHTML = "Error connecting to Stretch (see console for details)";
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', () => {
    document.getElementById('connection').innerHTML = "Disconnected";
    document.getElementById('camera').style.display = "none";
    document.getElementById('controls-container').style.display = "none";
    console.log('Connection to websocket server closed.');
});
