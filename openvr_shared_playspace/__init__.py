import os
import socket
from getpass import getuser
from queue import Queue, Empty
from threading import Thread

import openvr
import time

from struct import Struct

FRAMERATE = 90

MAXNAME = 200

class Device:

    def __init__(self, openvr_id):
        self._id = openvr_id
        self._x = 0
        self._y = 0
        self._z = 0
        self._pose = openvr.HmdMatrix34_t()

    pass
    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    def z(self):
        return self._z

    def update_pose(self, x=0, y=0, z=0):
        if self._id is not None:
            poses = openvr.VRSystem().getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseRawAndUncalibrated,
                                                                      0,
                                                                      self._id
                                                                      )
            pose = poses[self._id]

            self._x = pose.mDeviceToAbsoluteTracking[0][3]
            self._y = pose.mDeviceToAbsoluteTracking[1][3]
            self._z = pose.mDeviceToAbsoluteTracking[2][3]
            self._pose = pose.mDeviceToAbsoluteTracking
        else:
            self._x, self._y, self._z = x, y, z


class NetworkUpdateMsg:

    format_str = '!' + 'c' * MAXNAME + 'ddd'
    formater = Struct(format_str)
    length = formater.size

    @classmethod
    def from_device(cls, device: Device):
        name = (socket.gethostname() + '_' + getuser()).ljust(MAXNAME,)[0:MAXNAME]
        x = device.x
        y = device.y
        z = device.z
        args = [bytes(i.encode('utf-8')) for i in name][:MAXNAME]
        args.append(x)
        args.append(y)
        args.append(z)
        return cls.formater.pack(*args)

    @classmethod
    def from_network(cls, bytes_str):
        unpacked_items = cls.formater.unpack(bytes_str)
        name = ''.join([i.decode('utf-8') for i in unpacked_items[:MAXNAME]]).strip()
        x, y, z = unpacked_items[-3:]
        return name, x, y, z

def check_result(result):
    if result:
        error_name = openvr.VROverlay().getOverlayErrorNameFromEnum(result)
        raise Exception("OpenVR Error:", error_name)

class BroadcastSender:
    def __init__(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Set a timeout so the socket does not block
        # indefinitely when trying to receive data.
        server.settimeout(0.1)
        server.bind(("", 45368))
        self.server = server
    def put(self, dev: Device):
        message = NetworkUpdateMsg.from_device(dev)
        self.server.sendto(message, ('<broadcast>', 45368))

class BroadcastReceiver:
    def __init__(self):
        self._recv_queue = Queue()
        self._working = True

        self._recv_thread = Thread(target=self._recv)
        self._recv_thread.daemon = True
        self._recv_thread.start()


    def _recv(self):
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        client.bind(("", 45368))
        self._client = client
        while self._working:
            msg = self._client.recv(NetworkUpdateMsg.length)
            decoded = NetworkUpdateMsg.from_network(msg)
            self._recv_queue.put(decoded)

    def get(self):
        to_return = []
        try:

            to_return.append(self._recv_queue.get(block=False))
        except Empty:
            return to_return

class Update:
    @property
    def name(self):
        return ''
    @property
    def position(self):
        return [0, 0, 0]




class DeviceVisualiser:
    def __init__(self, name):
        self._name = name
        image_default_path = os.path.join(os.path.dirname(__file__), '..', 'images')
        image_file = os.path.join((image_default_path, name + '.png'))
        default_image_file = os.path.join((image_default_path, 'default.png'))
        if os.path.exists(image_file):
            self._image_file = self._image_file
        else:
            self._image_file = default_image_file

class SharedPlayspace:
    def __init__(self, vrsys):
        self.vrsys = vrsys
        self._devices_to_broadcast = {}
        self._devices_to_show_local = {}
        self._devices_to_show_remote = {}
        self._device_visualisers = {}
        self.sender = BroadcastSender()
        self.receiver = BroadcastReceiver()

    def get_headset(self):
        for i in range(16):
            if self.vrsys.getTrackedDeviceClass() == openvr.TrackedDeviceClass_HMD:
                return i

    def run(self):
        while True:
            t0 = time.monotonic()
            headset_id = self.get_headset()
            if headset_id not in self._devices_to_broadcast:
                self._devices_to_broadcast[headset_id] = Device(headset_id)
            self._update_and_send()
            self._receive_and_update()
            self._draw()

            left_to_sleep = 1 / FRAMERATE - (time.monotonic() - t0)
            if left_to_sleep > 0:
                time.sleep(left_to_sleep)

    def _update_and_send(self):
        for device in self._devices_to_broadcast.values():
            device.update_pose()
            self.sender.put(device)

    def _receive_and_update(self):
        updates = self.receiver.get()
        for update in updates:
            if update.name not in self._devices_to_show_remote:
                self._devices_to_show_remote[update.name] = Device(None)
            self._devices_to_show_remote[update.name].update_pose(update.position)

    def _draw(self):
        for i in self._devices_to_show_remote.values():
            print(i.x, i.y, i.z)


def main():
    while True:
        try:
            sys = openvr.init(openvr.VRApplication_Overlay)
            break
        except Exception:
            time.sleep(1)

    app = SharedPlayspace(vrsys=sys)
    app.run()
