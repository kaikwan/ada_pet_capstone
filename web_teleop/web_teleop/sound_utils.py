#!/usr/bin/env python3
from __future__ import print_function
import pyaudio
import wave
import usb.core
import struct
import os
import sys
from contextlib import contextmanager

import stretch_body.hello_utils as hu  # noqa F401

hu.print_stretch_re_use()


@contextmanager
def ignore_stderr():
    devnull = None
    try:
        devnull = os.open(os.devnull, os.O_WRONLY)
        stderr = os.dup(2)
        sys.stderr.flush()
        os.dup2(devnull, 2)
        try:
            yield
        finally:
            os.dup2(stderr, 2)
            os.close(stderr)
    finally:
        if devnull is not None:
            os.close(devnull)


# parameter list
# name: (id, offset, type, max, min , r/w, info)
PARAMETERS = {
    "AECFREEZEONOFF": (
        18,
        7,
        "int",
        1,
        0,
        "rw",
        "Adaptive Echo Canceler updates inhibit.",
        "0 = Adaptation enabled",
        "1 = Freeze adaptation, filter only",
    ),
    "AECNORM": (
        18,
        19,
        "float",
        16,
        0.25,
        "rw",
        "Limit on norm of AEC filter coefficients",
    ),
    "AECPATHCHANGE": (
        18,
        25,
        "int",
        1,
        0,
        "ro",
        "AEC Path Change Detection.",
        "0 = false (no path change detected)",
        "1 = true (path change detected)",
    ),
    "RT60": (18, 26, "float", 0.9, 0.25, "ro", "Current RT60 estimate in seconds"),
    "HPFONOFF": (
        18,
        27,
        "int",
        3,
        0,
        "rw",
        "High-pass Filter on microphone signals.",
        "0 = OFF",
        "1 = ON - 70 Hz cut-off",
        "2 = ON - 125 Hz cut-off",
        "3 = ON - 180 Hz cut-off",
    ),
    "RT60ONOFF": (18, 28, "int", 1, 0, "rw", "RT60 Estimation for AES. 0 = OFF 1 = ON"),
    "AECSILENCELEVEL": (
        18,
        30,
        "float",
        1,
        1e-09,
        "rw",
        "Threshold for signal detection in AEC [-inf .. 0] dBov (Default: -80dBov = 10log10(1x10-8))",
    ),
    "AECSILENCEMODE": (
        18,
        31,
        "int",
        1,
        0,
        "ro",
        "AEC far-end silence detection status. ",
        "0 = false (signal detected) ",
        "1 = true (silence detected)",
    ),
    "AGCONOFF": (
        19,
        0,
        "int",
        1,
        0,
        "rw",
        "Automatic Gain Control. ",
        "0 = OFF ",
        "1 = ON",
    ),
    "AGCMAXGAIN": (
        19,
        1,
        "float",
        1000,
        1,
        "rw",
        "Maximum AGC gain factor. ",
        "[0 .. 60] dB (default 30dB = 20log10(31.6))",
    ),
    "AGCDESIREDLEVEL": (
        19,
        2,
        "float",
        0.99,
        1e-08,
        "rw",
        "Target power level of the output signal. ",
        "[-inf .. 0] dBov (default: -23dBov = 10log10(0.005))",
    ),
    "AGCGAIN": (
        19,
        3,
        "float",
        1000,
        1,
        "rw",
        "Current AGC gain factor. ",
        "[0 .. 60] dB (default: 0.0dB = 20log10(1.0))",
    ),
    "AGCTIME": (
        19,
        4,
        "float",
        1,
        0.1,
        "rw",
        "Ramps-up / down time-constant in seconds.",
    ),
    "CNIONOFF": (
        19,
        5,
        "int",
        1,
        0,
        "rw",
        "Comfort Noise Insertion.",
        "0 = OFF",
        "1 = ON",
    ),
    "FREEZEONOFF": (
        19,
        6,
        "int",
        1,
        0,
        "rw",
        "Adaptive beamformer updates.",
        "0 = Adaptation enabled",
        "1 = Freeze adaptation, filter only",
    ),
    "STATNOISEONOFF": (
        19,
        8,
        "int",
        1,
        0,
        "rw",
        "Stationary noise suppression.",
        "0 = OFF",
        "1 = ON",
    ),
    "GAMMA_NS": (
        19,
        9,
        "float",
        3,
        0,
        "rw",
        "Over-subtraction factor of stationary noise. min .. max attenuation",
    ),
    "MIN_NS": (
        19,
        10,
        "float",
        1,
        0,
        "rw",
        "Gain-floor for stationary noise suppression.",
        "[-inf .. 0] dB (default: -16dB = 20log10(0.15))",
    ),
    "NONSTATNOISEONOFF": (
        19,
        11,
        "int",
        1,
        0,
        "rw",
        "Non-stationary noise suppression.",
        "0 = OFF",
        "1 = ON",
    ),
    "GAMMA_NN": (
        19,
        12,
        "float",
        3,
        0,
        "rw",
        "Over-subtraction factor of non- stationary noise. min .. max attenuation",
    ),
    "MIN_NN": (
        19,
        13,
        "float",
        1,
        0,
        "rw",
        "Gain-floor for non-stationary noise suppression.",
        "[-inf .. 0] dB (default: -10dB = 20log10(0.3))",
    ),
    "ECHOONOFF": (19, 14, "int", 1, 0, "rw", "Echo suppression.", "0 = OFF", "1 = ON"),
    "GAMMA_E": (
        19,
        15,
        "float",
        3,
        0,
        "rw",
        "Over-subtraction factor of echo (direct and early components). min .. max attenuation",
    ),
    "GAMMA_ETAIL": (
        19,
        16,
        "float",
        3,
        0,
        "rw",
        "Over-subtraction factor of echo (tail components). min .. max attenuation",
    ),
    "GAMMA_ENL": (
        19,
        17,
        "float",
        5,
        0,
        "rw",
        "Over-subtraction factor of non-linear echo. min .. max attenuation",
    ),
    "NLATTENONOFF": (
        19,
        18,
        "int",
        1,
        0,
        "rw",
        "Non-Linear echo attenuation.",
        "0 = OFF",
        "1 = ON",
    ),
    "NLAEC_MODE": (
        19,
        20,
        "int",
        2,
        0,
        "rw",
        "Non-Linear AEC training mode.",
        "0 = OFF",
        "1 = ON - phase 1",
        "2 = ON - phase 2",
    ),
    "SPEECHDETECTED": (
        19,
        22,
        "int",
        1,
        0,
        "ro",
        "Speech detection status.",
        "0 = false (no speech detected)",
        "1 = true (speech detected)",
    ),
    "FSBUPDATED": (
        19,
        23,
        "int",
        1,
        0,
        "ro",
        "FSB Update Decision.",
        "0 = false (FSB was not updated)",
        "1 = true (FSB was updated)",
    ),
    "FSBPATHCHANGE": (
        19,
        24,
        "int",
        1,
        0,
        "ro",
        "FSB Path Change Detection.",
        "0 = false (no path change detected)",
        "1 = true (path change detected)",
    ),
    "TRANSIENTONOFF": (
        19,
        29,
        "int",
        1,
        0,
        "rw",
        "Transient echo suppression.",
        "0 = OFF",
        "1 = ON",
    ),
    "VOICEACTIVITY": (
        19,
        32,
        "int",
        1,
        0,
        "ro",
        "VAD voice activity status.",
        "0 = false (no voice activity)",
        "1 = true (voice activity)",
    ),
    "STATNOISEONOFF_SR": (
        19,
        33,
        "int",
        1,
        0,
        "rw",
        "Stationary noise suppression for ASR.",
        "0 = OFF",
        "1 = ON",
    ),
    "NONSTATNOISEONOFF_SR": (
        19,
        34,
        "int",
        1,
        0,
        "rw",
        "Non-stationary noise suppression for ASR.",
        "0 = OFF",
        "1 = ON",
    ),
    "GAMMA_NS_SR": (
        19,
        35,
        "float",
        3,
        0,
        "rw",
        "Over-subtraction factor of stationary noise for ASR. ",
        "[0.0 .. 3.0] (default: 1.0)",
    ),
    "GAMMA_NN_SR": (
        19,
        36,
        "float",
        3,
        0,
        "rw",
        "Over-subtraction factor of non-stationary noise for ASR. ",
        "[0.0 .. 3.0] (default: 1.1)",
    ),
    "MIN_NS_SR": (
        19,
        37,
        "float",
        1,
        0,
        "rw",
        "Gain-floor for stationary noise suppression for ASR.",
        "[-inf .. 0] dB (default: -16dB = 20log10(0.15))",
    ),
    "MIN_NN_SR": (
        19,
        38,
        "float",
        1,
        0,
        "rw",
        "Gain-floor for non-stationary noise suppression for ASR.",
        "[-inf .. 0] dB (default: -10dB = 20log10(0.3))",
    ),
    "GAMMAVAD_SR": (
        19,
        39,
        "float",
        1000,
        0,
        "rw",
        "Set the threshold for voice activity detection.",
        "[-inf .. 60] dB (default: 3.5dB 20log10(1.5))",
    ),
    # 'KEYWORDDETECT': (20, 0, 'int', 1, 0, 'ro', 'Keyword detected. Current value so needs polling.'),
    "DOAANGLE": (
        21,
        0,
        "int",
        359,
        0,
        "ro",
        "DOA angle. Current value. Orientation depends on build configuration.",
    ),
}


class Tuning:
    TIMEOUT = 100000

    def __init__(self, dev):
        self.dev = dev

    def write(self, name, value):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        if data[5] == "ro":
            raise ValueError("{} is read-only".format(name))

        id = data[0]

        # 4 bytes offset, 4 bytes value, 4 bytes type
        if data[2] == "int":
            payload = struct.pack(b"iii", data[1], int(value), 1)
        else:
            payload = struct.pack(b"ifi", data[1], float(value), 0)

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            0,
            id,
            payload,
            self.TIMEOUT,
        )

    def read(self, name):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        id = data[0]

        cmd = 0x80 | data[1]
        if data[2] == "int":
            cmd |= 0x40

        length = 8

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            cmd,
            id,
            length,
            self.TIMEOUT,
        )

        if sys.version_info[:3] > (3, 0):
            response = struct.unpack(b"ii", response.tobytes())
        else:
            response = struct.unpack(b"ii", response.tostring())

        if data[2] == "int":
            result = response[0]
        else:
            result = response[0] * (2.0 ** response[1])

        return result

    def set_vad_threshold(self, db):
        self.write("GAMMAVAD_SR", db)

    @property
    def direction(self):
        return self.read("DOAANGLE")

    @property
    def version(self):
        return self.dev.ctrl_transfer(
            usb.util.CTRL_IN
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            0x80,
            0,
            1,
            self.TIMEOUT,
        )[0]

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)


def get_respeaker_device_id():
    with ignore_stderr():
        p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    num_devices = info.get("deviceCount")

    device_id = -1
    for i in range(num_devices):
        if (
            p.get_device_info_by_host_api_device_index(0, i).get("maxInputChannels")
        ) > 0:
            if "ReSpeaker" in p.get_device_info_by_host_api_device_index(0, i).get(
                "name"
            ):
                device_id = i

    return device_id


RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 6  # must flash 6_channels_firmware.bin first
RESPEAKER_WIDTH = 2
RESPEAKER_INDEX = get_respeaker_device_id()
CHUNK = 1024


def play_audio(file_path):
    wf = wave.open(file_path, "rb")
    p = pyaudio.PyAudio()

    stream = p.open(
        format=p.get_format_from_width(wf.getsampwidth()),
        channels=wf.getnchannels(),
        rate=wf.getframerate(),
        output=True,
    )

    chunk = 1024
    data = wf.readframes(chunk)
    while data:
        stream.write(data)
        data = wf.readframes(chunk)

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf.close()


if __name__ == "__main__":
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    try:
        if dev:
            respeaker = Tuning(dev)
            print("* playing audio")
            play_audio(
                "/home/hello-robot/ada_pet_capstone/src/ada_pet_capstone/web/audio/meow.wav"
            )
            print("* done")
    except usb.core.USBError:
        print("Respeaker not on USB bus")
