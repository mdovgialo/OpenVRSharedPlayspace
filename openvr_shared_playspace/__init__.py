import os
import socket
import traceback
from collections import namedtuple
from getpass import getuser
from queue import Queue, Empty
from threading import Thread

import openvr
import time

from struct import Struct

from math import cos, sin

FRAMERATE = 90

MAXNAME = 200

PORT = 45368

Update = namedtuple('Update', 'name position')

class Device:

    def __init__(self, openvr_id):
        self._id = openvr_id
        self._x = 0
        self._y = 0
        self._z = 0
        self._pose = openvr.HmdMatrix34_t()

    def __repr__(self):
        return "<Device x {} y {} z {}>".format(self.x, self.y, self.z)

    pass
    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    def update_pose(self, x=0, y=0, z=0):
        if self._id is not None:
            poses = openvr.VRSystem().getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseRawAndUncalibrated,
                                                                      0,
                                                                      16
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
        args.append(float(x))
        args.append(float(y))
        args.append(float(z))
        return cls.formater.pack(*args)

    @classmethod
    def from_network(cls, bytes_str):
        unpacked_items = cls.formater.unpack(bytes_str)
        name = ''.join([i.decode('utf-8') for i in unpacked_items[:MAXNAME]]).strip()
        x, y, z = unpacked_items[-3:]
        return Update(name, [x, y, z])




def check_result(result):
    if result:
        error_name = openvr.VROverlay().getOverlayErrorNameFromEnum(result)
        raise Exception("OpenVR Error:", error_name)

class BroadcastSender:
    def __init__(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Set a timeout so the socket does not block
        # indefinitely when trying to receive data.
        server.settimeout(0.1)
        server.bind(("", PORT))
        self.server = server
    def put(self, dev: Device):
        message = NetworkUpdateMsg.from_device(dev)
        self.server.sendto(message, ('<broadcast>', PORT))

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
        client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        client.bind(("", PORT))
        self._client = client
        while self._working:
            msg = self._client.recv(NetworkUpdateMsg.length)
            decoded = NetworkUpdateMsg.from_network(msg)
            self._recv_queue.put(decoded)

    def get(self):
        to_return = []
        while True:
            try:
                to_return.append(self._recv_queue.get(block=False))
            except Empty:
                return to_return

class DeviceVisualiser:
    def __init__(self, name):
        self._name = name
        image_default_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'images'))
        image_file = os.path.join(image_default_path, name + '.png')
        default_image_file = os.path.join(image_default_path, 'default.png')
        print("Looking for a SharedPlayspace avatar in {}".format(image_file))
        if os.path.exists(image_file):
            print("Found it!")
            self._image_file = self._image_file
        else:
            print("Not found! Using default! {}".format(default_image_file))
            self._image_file = default_image_file

        self.sys = openvr.VRSystem()
        self.vroverlay = openvr.IVROverlay()
        result, self.ovr = self.vroverlay.createOverlay(self._name.encode(), self._name.encode())
        check_result(self.vroverlay.setOverlayColor(self.ovr, 1, 1, 1))
        check_result(self.vroverlay.setOverlayAlpha(self.ovr, 1))
        check_result(self.vroverlay.setOverlayWidthInMeters(self.ovr, 0.3))
        check_result(self.vroverlay.setOverlayFromFile(self.ovr, self._image_file.encode()))
        check_result(self.vroverlay.showOverlay(self.ovr))

    def update(self, headset_pose, x, y, z):
        h_x = headset_pose.m[0][3]
        h_y = headset_pose.m[1][3]
        h_z = headset_pose.m[2][3]

        dst = ((h_x - x)**2  + (h_y - y)**2 + (h_z - z)**2)**0.5
        min_dst = 1.0
        max_dst = 2.0

        min_alpha = 0.3
        max_alpha = 0.01

        if dst < min_dst:
            dst = min_dst
        if dst > max_dst:
            dst = max_dst

        dst_norm =  1 - (dst - min_dst) / max_dst
        alpha = (dst_norm * (min_alpha - max_alpha)) + max_alpha

        rotate = initRotationMatrix(0, 0)
        transform = matMul33(rotate, headset_pose)

        transform.m[0][3] = float(x)
        transform.m[1][3] = float(y)
        transform.m[2][3] = float(z)

        ulOverlayHandle = self.ovr
        eTrackingOrigin = openvr.TrackingUniverseRawAndUncalibrated
        fn = self.vroverlay.function_table.setOverlayTransformAbsolute
        pmatTrackingOriginToOverlayTransform = transform
        result = fn(ulOverlayHandle, eTrackingOrigin, openvr.byref(pmatTrackingOriginToOverlayTransform))
        check_result(result)
        check_result(self.vroverlay.setOverlayAlpha(self.ovr, alpha))

def matMul33(a, b, result=None):
    if result is None:
        result = openvr.HmdMatrix34_t()
    for i in range(3):
        for j in range(3):
            result.m[i][j] = 0.0
            for k in range(3):
                result.m[i][j] += a.m[i][k] * b.m[k][j]
    result[0][3] = b[0][3]
    result[1][3] = b[1][3]
    result[2][3] = b[2][3]
    return result

def initRotationMatrix(axis, angle, matrix=None):
    # angle in radians
    if matrix is None:
        matrix = openvr.HmdMatrix34_t()
    if axis==0:
        matrix.m[0][0] = 1.0
        matrix.m[0][1] = 0.0
        matrix.m[0][2] = 0.0
        matrix.m[0][3] = 0.0
        matrix.m[1][0] = 0.0
        matrix.m[1][1] = cos(angle)
        matrix.m[1][2] = -sin(angle)
        matrix.m[1][3] = 0.0
        matrix.m[2][0] = 0.0
        matrix.m[2][1] = sin(angle)
        matrix.m[2][2] = cos(angle)
        matrix.m[2][3] = 0.0
    elif axis==1:
        matrix.m[0][0] = cos(angle)
        matrix.m[0][1] = 0.0
        matrix.m[0][2] = sin(angle)
        matrix.m[0][3] = 0.0
        matrix.m[1][0] = 0.0
        matrix.m[1][1] = 1.0
        matrix.m[1][2] = 0.0
        matrix.m[1][3] = 0.0
        matrix.m[2][0] = -sin(angle)
        matrix.m[2][1] = 0.0
        matrix.m[2][2] = cos(angle)
        matrix.m[2][3] = 0.0
    elif axis == 2:
        matrix.m[0][0] = cos(angle)
        matrix.m[0][1] = -sin(angle)
        matrix.m[0][2] = 0.0
        matrix.m[0][3] = 0.0
        matrix.m[1][0] = sin(angle)
        matrix.m[1][1] = cos(angle)
        matrix.m[1][2] = 0.0
        matrix.m[1][3] = 0.0
        matrix.m[2][0] = 0.0
        matrix.m[2][1] = 0.0
        matrix.m[2][2] = 1.0
        matrix.m[2][3] = 0.0
    return matrix

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
        device_class = openvr.TrackedDeviceClass_HMD
        for i in range(16):

            if self.vrsys.getTrackedDeviceClass(i) == device_class:
                return i

    def get_tracker(self):
        device_class = openvr.TrackedDeviceClass_GenericTracker
        for i in range(16):

            if self.vrsys.getTrackedDeviceClass(i) == device_class:
                return i

    def run(self):
        while True:
            t0 = time.monotonic()
            headset_id = self.get_headset()
            if headset_id not in self._devices_to_broadcast:
                if headset_id is None:
                    headset_id = self.get_tracker()
                self._devices_to_broadcast[headset_id] = Device(headset_id)
            self._update_and_send()
            self._receive_and_update()
            self._draw(headset_id)

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
            if update.name.startswith(socket.gethostname()):
                continue
            if update.name not in self._devices_to_show_remote:
                self._devices_to_show_remote[update.name] = Device(None)
            self._devices_to_show_remote[update.name].update_pose(*update.position)

    def _draw(self, headset_id):
        poses = openvr.VRSystem().getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseRawAndUncalibrated,
                                                                  0,
                                                                  16
                                                                  )
        pose = poses[headset_id]

        headset_pose = pose.mDeviceToAbsoluteTracking
        for name, i in self._devices_to_show_remote.items():
            if name not in self._device_visualisers:
                self._device_visualisers[name] = DeviceVisualiser(name)
            self._device_visualisers[name].update(headset_pose, i.x, i.y, i.z)


def main():
    while True:
        try:
            sys = openvr.init(openvr.VRApplication_Overlay)
            break
        except Exception as e:
            traceback.print_exc()
            time.sleep(1)

    app = SharedPlayspace(vrsys=sys)
    app.run()

if __name__ == '__main__':
    main()