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