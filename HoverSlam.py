import krpc
from time import sleep
import numpy as np


class HoverSlam:
    def __init__(self, vessel=None):
        self.conn = krpc.connect('HoverSlam')
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel if vessel == None else vessel
        self.body = self.vessel.orbit.body
        self.body_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.flight = self.vessel.flight(self.body_ref)
        self.target = self.space_center.target_vessel
        self.drawing = self.conn.drawing
        
        if self.target is None:
            print('Selecione um alvo!')
            exit()

        # Streams
        self.vertical_speed = self.conn.add_stream(getattr, self.flight, "vertical_speed")
        self.surface_altitude = self.conn.add_stream(getattr, self.flight, "surface_altitude")


        # Params
        self.ag = self.body.surface_gravity


        # Initializing
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.engage()


        # Drawing
        self.line_lenght = 10
        self.target_dir_draw = self.drawing.add_direction((0, 0, 0), self.surface_ref)
        self.prograde_dir_draw = self.drawing.add_direction((0, 0, 0), self.surface_ref)
        self.target_dir_draw.color = (255, 0, 0)
        self.prograde_dir_draw.color = (0, 0, 255)


        # Main Loop
        while True:
            target_pos = np.array(self.target.position(self.surface_ref))
            target_dir = self.normalize(target_pos)
            prograde_dir = self.normalize(np.array(self.velocity(self.vessel, self.surface_ref)))
            error_dir = target_dir - prograde_dir

            self.target_dir_draw.end = target_dir * self.line_lenght
            self.prograde_dir_draw.end = prograde_dir * self.line_lenght

            aim_dir = [1, 0, 0] + error_dir

            print(error_dir)

            aeng = self.vessel.available_thrust / self.vessel.mass
            throttle_cancel_ag = (self.ag / aeng)

            throttle = np.linalg.norm(error_dir)
            self.vessel.control.throttle = throttle + (throttle_cancel_ag if error_dir[0] > 0 else 0)
            #self.vessel.control.throttle = throttle + (throttle_cancel_ag if error_dir[0] > 0 else -throttle_cancel_ag)

            # Limitar o pitch para 10Deg

            self.aim_pos(aim_dir)

        

    def aim_pos(self, dir):
        self.vessel.auto_pilot.target_direction = dir



    def velocity(self, vessel, ref_frame): # Get velocity in self reference
        return self.space_center.transform_direction(vessel.velocity(self.body_ref), self.body_ref, ref_frame)

    def normalize(self, vector):
        return vector / np.linalg.norm(vector)

    def altitude(self):
        return max(0, self.surface_altitude() + self.vessel.bounding_box(self.surface_ref)[0][0])
    

if __name__ == '__main__':
    HoverSlam()