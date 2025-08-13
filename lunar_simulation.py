import json
import numpy as np
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# class for Lunar lanidng and launch environment
class Lunar_landing_simulation:
    '''Lunar_landing_simulation is a class that simulates a spacecraft launch.'''

    def __init__(self, spacecraft_information, launch_condition):
        '''Lunar_landing_simulation(spacecraft_information, launch_condition) -> Lunar_landing_simulation
        Initalizes a spacecraft launch simulation.
        spacecraft_information is a JSON of the spacecraft performance.
        launch_condition is a dictionary of the launch conditions.'''
        # get spacecraft performance infromation
        self.load_spacecraft_information(spacecraft_information)

        # load launch condition
        self.launch_inclination = math.radians(launch_condition["inclination"]) # launch orbit inclination

        # enivornment constants

        self.mu = (6.6743 * (10 ** -11)) * (7.34767309  * (10 ** 22))
        self.radius_moon  = 1737000



        self.reset()



    def load_spacecraft_information(self, spacecraft_information):
        '''load_spacecraft_information(spacecraft_information) -> None
        sets the spacecraft performance based on spacecraft_information JSON'''
        # open the file
        spacecraft_file = open(spacecraft_information)

        self.spacecraft_dictionary = json.load(spacecraft_file) # load the JSON

        
        self.present_mass = self.spacecraft_dictionary["Spacecraft Mass"]

        # define the spacecraft stage names
        self.number_of_thrusters = len(self.spacecraft_dictionary["Thrusters"])
        
        # calculate the torque outputed to the vehicle when it is engaged
        for thruster_number in range(self.number_of_thrusters):
            #find lever arm distance
            x_pos = self.spacecraft_dictionary["Thrusters"][thruster_number]["x Position"]
            # lever_arm_dist = math.sqrt(x_pos**2 + self.spacecraft_dictionary["Thrusters"][thruster_number]["y Position"]**2)
            self.spacecraft_dictionary["Thrusters"][thruster_number]["Torque"] = x_pos * self.spacecraft_dictionary["Thrusters"][thruster_number]["Thrust"] # lever_arm_dist * x_pos * self.spacecraft_dictionary["Thrusters"][thruster_number]["Thrust"] / lever_arm_dist


        self.moment_of_inertia = self.spacecraft_dictionary["Moment of Inertia"]

        spacecraft_file.close()
                


        
    def mass(self, time, height):
        '''Lunar_landing_simulation.mass(time, height) -> float
        returns the mass of the spacecraft depending on the time and height'''

    
        return self.present_mass




    def run_an_period(self, run_time, thruster_settings, delta_time = 0.1):
        '''Lunar_landing_simulation.run_an_period(run_time, pitch_angle, gravity_turn, delta_time = 0.1) -> None
        runs the simulation for the given period at the given pitch_angle.'''

        for incrament in range(int(run_time/delta_time)):
            self.time += delta_time

            
            

            g = self.mu/((self.radius_moon + self.height) ** 2 ) # gravitaion acceleration

            # make sure thruster settings present an 
            self.thrust_force = 0
            net_torque = 0
            for thruster_number in range(self.number_of_thrusters):
                #find lever arm distance
                thruster_setting = thruster_settings[thruster_number]
                if thruster_setting != 0:
                    if thruster_setting < self.spacecraft_dictionary["Thrusters"][thruster_number]["Throttle Range"][0]:
                        thruster_setting = self.spacecraft_dictionary["Thrusters"][thruster_number]["Throttle Range"][0]
                    elif  thruster_setting > self.spacecraft_dictionary["Thrusters"][thruster_number]["Throttle Range"][1]:
                        thruster_setting = self.spacecraft_dictionary["Thrusters"][thruster_number]["Throttle Range"][1]
                
                # find the added torque for each thruster
                net_torque += thruster_setting * self.spacecraft_dictionary["Thrusters"][thruster_number]["Torque"]
                # add the thrust to total thrust
                self.thrust_force += thruster_setting * self.spacecraft_dictionary["Thrusters"][thruster_number]["Thrust"]

                self.thruster_list[thruster_number].append(thruster_setting)
                # print(thruster_number)
            
            self.angular_acceleration = net_torque / self.moment_of_inertia


                
            # update acceleration 
            self.acceleration = np.array([
                (math.cos(self.pitch_angle ) * (self.thrust_force/self.present_mass)) + (g * math.cos(self.degree_position + math.pi)), 
                (math.sin(self.pitch_angle) * (self.thrust_force/self.present_mass)) + (g * math.sin(self.degree_position + math.pi))
                ])

            self.total_acceleration = np.linalg.norm(self.acceleration)


            # find angular velocity
            self.angular_velocity += self.angular_acceleration * delta_time

            self.pitch_angle += self.angular_velocity * delta_time
                

                
            # update actual speed
            # print(self.acceleration)
            self.speed += self.acceleration * delta_time

            self.total_speed =  np.linalg.norm(self.speed)



            # update position
            self.position += self.speed * delta_time

            # update height
            self.height = np.linalg.norm(self.position) - self.radius_moon

            # update flight angle
            self.flight_angle = np.arctan(self.speed[1]/self.speed[0])
            
            if self.speed[0] < 0:
                self.flight_angle += math.pi
            




            # get the distance from the launch site in terms of the surface of earth
            self.downrange_distance = (self.starting_degree_postion - self.degree_position) * self.radius_moon

            self.downrange_list.append(self.downrange_distance / 1000)

            # save new recording value
            self.time_list.append(self.time)

            self.height_list.append(self.height/1000)
           
            self.total_acceleration_list.append(self.total_acceleration)
            self.total_speed_list.append(self.total_speed)
            self.speed_list.append(self.speed)

            # self.position_list.append(self.position) 
            # print(self.position_list.shape)
            self.position_x_list.append(self.position[0]) 
            self.position_y_list.append(self.position[1]) 


            
            self.mass_list.append(self.present_mass)
            self.thrust_list.append(self.thrust_force)

            self.pitch_angle_list.append(math.degrees(self.pitch_angle))

            self.flight_angle_list.append((math.degrees(self.flight_angle)+ 90) % 360 - 90)
            

            # check if hit the ground
            if self.height < 0:
                self.hit_ground = True
                break



    def get_states(self):
        '''Rocekt_launch_simulation.get_states() -> list
        returns an array of the spacecrafts state'''
        return [self.time, self.acceleration[0], self.acceleration[1], self.speed[0], self.speed[1], self.flight_angle, self.angular_acceleration, self.angular_velocity, self.pitch_angle, self.height]
    

    def step(self, thruster_settings):
        '''Rocekt_launch_simulation.step() -> list
        returns the new state given a step'''
        self.run_an_period(0.1, thruster_settings)
        return self.get_states()




        


    

    def display_data(self):
        '''Rocekt_launch_simulation.display_data() -> list
        display the data.'''
        figure, axis = plt.subplots(2, 5) 

        axis[0,0].plot(self.time_list, self.height_list)
        # axis[0,0].plot(tList, hTestList)
        axis[0,0].set_xlabel('t (s)')
        axis[0,0].set_ylabel('h (km)')
        axis[0,0].legend(['x', 'y'], shadow=True)
        axis[0,0].set_title('spacecraft Height')

        axis[0,1].plot(self.time_list, self.total_speed_list)
        axis[0,1].set_xlabel('t (s)')
        axis[0,1].set_ylabel('v (m/s)')
        axis[0,1].legend(['x', 'y'], shadow=True)
        axis[0,1].set_title('spacecraft Velocity')

        axis[1,0].plot(self.time_list, self.total_acceleration_list)
        axis[1,0].set_xlabel('t (s)')
        axis[1,0].set_ylabel('a (m/s^2)')
        axis[1,0].legend(['x', 'y'], shadow=True)
        axis[1,0].set_title('spacecraft Acceleration')

        axis[1,1].plot(self.time_list, self.thrust_list)
        axis[1,1].set_xlabel('t (s)')
        axis[1,1].set_ylabel('Force (N)')
        axis[1,1].legend(['x', 'y'], shadow=True)
        axis[1,1].set_title('Thrust')

        axis[0,2].plot(self.time_list, self.mass_list)
        axis[0,2].set_xlabel('t (s)')
        axis[0,2].set_ylabel('Mass (kg)')
        axis[0,2].legend(['x', 'y'], shadow=True)
        axis[0,2].set_title('spacecraft Mass')

        axis[1,2].plot(self.time_list, self.flight_angle_list)
        axis[1,2].set_xlabel('t (s)')
        axis[1,2].set_ylabel('Angle (Deg)')
        axis[1,2].legend(['x', 'y'], shadow=True)
        axis[1,2].set_title('Flight Angle')

        axis[0,3].plot(self.time_list, self.pitch_angle_list)
        axis[0,3].set_xlabel('t (s)')
        axis[0,3].set_ylabel('Angle (Deg)')
        axis[0,3].legend(['x', 'y'], shadow=True)
        axis[0,3].set_title('Pitch Angle')

        #earth

        center = (0, 0)
        radius = self.radius_moon
        circle = Circle(center, radius, color='black', fill=False)

        # axis[1,3].set_aspect('equal')
        # axis[1,3].add_patch(circle)
        axis[1,3].plot(self.position_x_list, self.position_y_list)
        axis[1,3].set_title('Orbit')

        for thruster in self.thruster_list:

            axis[0,4].plot(self.time_list, thruster)
        axis[0,4].set_xlabel("Downrange (km)")
        axis[0,4].set_ylabel("Height (km)")
        axis[0,4].set_title('Trajectory')




        plt.show()

    def reset(self, initial_condition = []):
         # initalize simulation variables
        self.time = 0
        self.hit_ground = False

        self.starting_degree_postion = math.pi/2

        self.degree_position = self.starting_degree_postion # this is the angle in the 2D plane of the orbit
        
        # height
        self.height = 1000

        # angular on xy plane
        self.angular_acceleration = 0 
        self.angular_velocity = 0

        # acceleration
        self.acceleration = np.array([0.0, -self.mu/((self.radius_moon + self.height) ** 2 )])

        self.polar_acceleration = np.array([0.0, -self.mu/((self.radius_moon + self.height) ** 2 )])

        self.total_acceleration = 0

        # initalize the inital speed of the spacecraft. Factor in earth rotation
        self.speed = np.array([
            0.0,
            0.0
            ], dtype=np.float64)


        self.total_speed = np.sum(self.speed)


        self.flight_angle = math.pi/2

        # self.flight_angle_earth = math.pi/2
        # self.flight_angle_space = math.pi/2
        # self.cartesian_flight_angle = 0.0
        self.pitch_angle =  math.pi/2
        # self.pitch_angle =  -math.pi/2
        # self.height = 0.0
        self.downrange_distance = 0.0 
        self.downrange_list = []

        # spacecraft values
        
        self.thrust_force = 0.0 # thrust

        self.present_mass = self.mass(0, 0) # mass

        # position
        self.position = np.array([math.cos(self.degree_position) *(self.radius_moon + self.height), math.sin(self.degree_position) *(self.radius_moon + self.height)])


        # no more holds
        self.reach_hold = False

        # value recording list
        self.position_x_list= []
        self.position_y_list= []
        self.height_list = []
        self.total_acceleration_list = []
        self.speed_list = []
        self.total_speed_list = []
        self.time_list = []
        self.flight_angle_list = []
        self.pitch_angle_list = []
        self.degree_position_list = []
        self.thrust_list = []
        self.mass_list = []
        # print(self.number_of_thrusters)
        self.thruster_list = [[] for _ in range(self.number_of_thrusters)]
        # print(self.thruster_list)

        # area covered per second
        self.area_covered_per_second = 0

        self.area_covered_every_tenth_seconds = list(range(10))



        return self.get_states()


# import os

# # file_name = "Starship2_new.json"
# file_name = "blue_ghost.json"
# # file_name = "SLS.json"

# test = Lunar_landing_simulation(file_name, {"inclination": 0.453786})


# # 2 Burn 700kms
# # pitch_target = {24: [math.radians(81), False], 40: [math.radians(45), False], 115:  [math.radians(25), False], 135:  [math.radians(30), False], 265: [math.radians(16.3), True], 500: [math.radians(5.11), True]}

# # pitch_target = {40: [math.radians(81), False], 80: [math.radians(45), False], 115:  [math.radians(25), False], 135:  [math.radians(30), False], 250: [math.radians(18), True], 400: [math.radians(5.11), True]}

# #2 Burn Expendable
# pitch_target = {24: [math.radians(81), False], 40: [math.radians(45), False], 115:  [math.radians(25), False], 135:  [math.radians(30), False], 265: [math.radians(16.3), True], 500: [math.radians(5.11), True]}

# state_name = ["time", "thrust", "mass", "acceleration x", "acceleration y", "speed x", "speed y","flight angle", "pitch angle", "degree position", "height", "downrange distance"]



# pitch_angle = math.radians(90)
# on_hold = False
# import random
# # reach_hold_time = 453
# reach_hold_time = 1500
# # for time in range(0, 7):
# pitch_angle_command = [math.radians(90), False]
# # for time in range(0, 2700000):
# for time in range(0, 100):
#         # gravity_turn = True
#     states = test.step([random.choice([0,1]), random.choice([0,1]),random.choice([0,1]),random.choice([0,1])])




# test.display_data()
