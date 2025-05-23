#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import usb.core
from .sound_utils import Tuning, play_audio
from std_msgs.msg import String


class PlaySoundSubscriber(Node):
    def __init__(self):
        super().__init__("play_sound_subscriber")
        self.subscription = self.create_subscription(
            String, "play_sound", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        self.respeaker = Tuning(self.dev)

    def listener_callback(self, msg):
        try:
            if self.dev:
                if msg.data == "meow":
                    play_audio(
                        "/home/hello-robot/ada_pet_capstone/src/ada_pet_capstone/web/audio/meow.wav"
                    )
        except usb.core.USBError:
            print("Respeaker not on USB bus")


def main(args=None):
    rclpy.init(args=args)
    play_sound_subscriber = PlaySoundSubscriber()

    rclpy.spin(play_sound_subscriber)

    play_sound_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
