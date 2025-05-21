function playSound(sound) {
    var playSound = new ROSLIB.Topic({
    ros : ros,
    name : '/play_sound',
    messageType : 'std_msgs/String'
    });

    var String = new ROSLIB.Message({
        data : sound
    });
    playSound.publish(String);
}
