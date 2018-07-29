import openvr
import time


FRAMERATE = 90

class BroadcastSender:
    pass
    def put(self, dev: DeviceBase):
        pass

class BroadcastReceiver:
    def get(self):
        pass
    pass

class Device:

    def __init__(self, openvr_id):
        self._id = openvr_id

    pass
    @property
    def x(self):
        return 0

    @property
    def y(self):
        return 0

    def z(self):
        return 0

class DeviceVisualiser:
    pass

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

            left_to_sleep = 1 / FRAMERATE(time.monotonic() - t0)
            if left_to_sleep > 0:
                time.sleep(left_to_sleep)


def main():
    while True:
        try:
            sys = openvr.init(openvr.VRApplication_Overlay)
            break
        except Exception:
            time.sleep(1)

    app = SharedPlayspace(vrsys=sys)
    app.run()
