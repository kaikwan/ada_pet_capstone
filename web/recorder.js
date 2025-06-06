let keyframes = [];

const recordKeyframe = () => {
    const yaw = currentJointPositions['yaw'];
    const pitch = currentJointPositions['pitch'];
    const roll = currentJointPositions['roll'];
    const lift = currentControlPositions['lift'];
    const arm = currentControlPositions['arm'];

    keyframes.push({
        yaw: yaw,
        pitch: pitch,
        roll: roll,
        lift: lift,
        arm: arm,
    });

    console.log('Keyframe recorded:', keyframes);

}
const resetKeyframes = () => {
    keyframes = [];
    console.log('Keyframes reset:', keyframes);
}

const playbackKeyframes = () => {
    if (keyframes.length === 0) {
        console.log('No keyframes to playback');
        return;
    }

    for (let i = 0; i < keyframes.length; i++) {
        const keyframe = keyframes[i];
        setTimeout(() => {
            executeFollowJointTrajectory(
                ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_lift', 'wrist_extension'],
                [keyframe.yaw, keyframe.pitch, keyframe.roll, keyframe.lift, keyframe.arm]
            )
        }, i * 1000); // Adjust the delay as needed
        console.log('Keyframe playback:', keyframe);
    }
}

let loop;

const loopKeyframes = () => {
    if (keyframes.length === 0) {
        console.log('No keyframes to loop');
        return;
    }

    let i = 0;
    loop = setInterval(() => {
        if (i >= keyframes.length) {
            i = 0; // Reset to the first keyframe
        }
        const keyframe = keyframes[i];
        executeFollowJointTrajectory(
            ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_lift', 'wrist_extension'],
            [keyframe.yaw, keyframe.pitch, keyframe.roll, keyframe.lift, keyframe.arm]
        );
        console.log('Keyframe loop:', keyframe);
        i++;
    }, 1000); // Adjust the delay as needed
}
const stopKeyframeLoop = () => {
    clearInterval(loop);
    console.log('Keyframe loop stopped');
}

const quickStow = () => {
    const stowPose =
        {
            yaw: 0,
            pitch: -0.63,
            roll: 0,
            lift: 0.3,
            arm: 0
          }
    executeFollowJointTrajectory(
        ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_lift', 'wrist_extension'],
        [stowPose.yaw, stowPose.pitch, stowPose.roll, stowPose.lift, stowPose.arm]
    );

    console.log('Quick stow pose executed:', stowPose);

    // wait and then set yaw to -1.57
    setTimeout(() => {
        executeFollowJointTrajectory(
            ['joint_wrist_yaw'],
            [-1.57] // Set yaw to -1.57
        );
        console.log('Yaw set to -1.57');
    }, 3000); // Adjust the delay as needed
}

const pregraspToy = () => {
    const pregraspPose =
        {
            yaw: 0,
            pitch: -0.63,
            roll: -0.0015339807878856412,
            lift: 0.6531000656439779,
            arm: 0.04095989171186181,
            gripper_aperture: 0.1
          }
    executeFollowJointTrajectory(
        ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_lift', 'wrist_extension', 'gripper_aperture'],
        [pregraspPose.yaw, pregraspPose.pitch, pregraspPose.roll, pregraspPose.lift, pregraspPose.arm, pregraspPose.gripper_aperture]
    );
    console.log('Pregrasp pose executed:', pregraspPose);

    // wait and then set yaw to 0.
    setTimeout(() => {
        executeFollowJointTrajectory(
            ['joint_wrist_pitch'],
            [0] // Set yaw to 0
        );
        console.log('Yaw set to 0');
    }, 3000); // Adjust the delay as needed

}


const graspToy = () => {
    const graspPose =
        {
            yaw: 0,
            pitch: -0.013805827090970769,
            roll: -0.0015339807878856412,
            lift: 0.6531000656439779,
            arm: 0.3095989171186181
          }
    executeFollowJointTrajectory(
        ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_lift', 'wrist_extension'],
        [graspPose.yaw, graspPose.pitch, graspPose.roll, graspPose.lift, graspPose.arm]
    );

    // wait and then close gripper
    setTimeout(() => {
        executeFollowJointTrajectory(
            ['gripper_aperture'],
            [0] // Close gripper
        );
        console.log('Grasp pose executed:', graspPose);
    }, 3000); // Adjust the delay as needed
}

