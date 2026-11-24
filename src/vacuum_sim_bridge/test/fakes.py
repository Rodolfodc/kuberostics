"""Fake SimClient for tests (no ZMQ, no CoppeliaSim)."""


class FakeSim:
    def __init__(self):
        self.objects = {}
        self.arrays = {}
        self.pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.joints = {}
        self.sim_time = 0.0
        self.wheel_writes = []
        self.rgb = None
        self.depth = None
        self.fail = False

    def _check(self):
        if self.fail:
            raise RuntimeError('simulator down')

    def get_object(self, path):
        self._check()
        return self.objects.setdefault(path, len(self.objects) + 1)

    def get_float_array(self, handle, prop):
        self._check()
        return list(self.arrays.get(prop, []))

    def get_object_pose(self, handle):
        self._check()
        return self.pose

    def get_joint_position(self, handle):
        self._check()
        return self.joints.get(handle, 0.0)

    def get_sim_time(self):
        self._check()
        return self.sim_time

    def set_wheel_speeds(self, robot, left, right):
        self._check()
        self.wheel_writes.append((robot, left, right))

    def read_rgb(self, handle):
        self._check()
        return self.rgb

    def read_depth(self, handle):
        self._check()
        return self.depth
