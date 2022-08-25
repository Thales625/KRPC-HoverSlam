import krpc
from time import sleep
import numpy as np
from math import sin, atan, radians, degrees

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
        self.pitch = self.conn.add_stream(getattr, self.vessel.flight(self.surface_ref), "pitch")


        # Params
        self.ag = self.body.surface_gravity
        self.final_speed = -3
        self.aproximating_speed = 10


        # Initializing
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.engage()
        #self.vessel.auto_pilot.reference_frame = self.surface_ref
        #self.vessel.auto_pilot.stopping_time = (0.1, 0.1, 0.1)
        #self.vessel.auto_pilot.deceleration_time = (1, 1, 1)
        self.vessel.auto_pilot.target_roll = -90
        self.vessel.auto_pilot.reference_frame = self.body_ref



        # Drawing
        self.line_lenght = 10
        self.target_dir_draw = self.drawing.add_direction((0, 0, 0), self.surface_ref)
        self.prograde_dir_draw = self.drawing.add_direction((0, 0, 0), self.surface_ref)
        self.aim_dir_draw = self.drawing.add_direction((0, 0, 0), self.surface_ref)
        self.target_dir_draw.color = (255, 0, 0)
        self.prograde_dir_draw.color = (0, 0, 255)
        self.aim_dir_draw.color = (0, 255, 0)


        # Main Loop
        while True:
            sleep(0.01)
            target_pos = np.array(self.target.position(self.surface_ref))
            target_dir = self.normalize(target_pos)
            prograde_dir = self.normalize(np.array(self.velocity(self.vessel, self.surface_ref)))
            error_dir = target_dir - prograde_dir

        
            #target_speed = self.target.velocity(self.surface_ref)
            #target_hor_speed = np.linalg.norm(target_speed[1:])

            aim_dir = [2, 0, 0] + error_dir/2

            #aim_dir = self.limit_pitch(aim_dir, 10)

            self.aim_pos(aim_dir)

            aeng = self.vessel.available_thrust / self.vessel.mass
            throttle= (self.ag + (self.final_speed - self.vertical_speed()) * 5) / (aeng * sin(radians(self.pitch())))
            self.vessel.control.throttle = throttle

            # Usar o aim_dir[0] para controlar velocidade de aproximação


            # Drawing
            self.target_dir_draw.end = target_dir * self.line_lenght
            self.prograde_dir_draw.end = prograde_dir * self.line_lenght
            self.aim_dir_draw.end = aim_dir * self.line_lenght

            # Limitar velocidade vertical usando a gravidade
            # Limitar a velocidade horizontal multiplicando o error_dir
            #aim = self.direction_controller.calcule(aim_dir)

    
    def limit_pitch(self, dir, pitch):
        h = np.sqrt(np.sum(dir[1:]**2))
        actual_pitch = atan(h/dir[0])

        print(degrees(actual_pitch))
        return dir

    def aim_pos(self, dir):
        self.vessel.auto_pilot.target_direction = self.space_center.transform_direction(dir, self.surface_ref, self.body_ref)

    def velocity(self, vessel, ref_frame): # Get velocity in self reference
        return self.space_center.transform_direction(vessel.velocity(self.body_ref), self.body_ref, ref_frame)

    def normalize(self, vector):
        return vector / np.linalg.norm(vector)

    def altitude(self):
        return max(0, self.surface_altitude() + self.vessel.bounding_box(self.surface_ref)[0][0])
    

if __name__ == '__main__':
    HoverSlam()