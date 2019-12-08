import olympe


class Anafi_drone:
    def __init__(self, D_IP):
        self.D_IP = D_IP
        self.drone = olympe.Drone(self.D_IP, loglevel=0)
        self.drone.connection()
