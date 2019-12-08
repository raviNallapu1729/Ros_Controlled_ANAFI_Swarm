import olympe


class Anafi_drone:
    def __init__(self, D_IP):
        self.D_IP = D_IP
        self.drone = olympe.Drone(self.D_IP, loglevel=0)
        self.drone.connection()


class Waypoint():
    def __init__(self, x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz):
        self.x = x
        self.y = y
        self.z = z

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.vx = vx
        self.vy = vy
        self.vz = vz

        self.wx = wx
        self.wy = wy
        self.wz = wz