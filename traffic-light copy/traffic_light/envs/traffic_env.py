import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from gym.spaces import Discrete, Box
import random

car_ID = 0
reward = 0
done = False
courses_list = ['straight', 'right', 'left']
origin_list = ['North', 'East', 'South', 'West']


class TrafficLightEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        # ACTIONS 4 are red light w green in other, red w green left, green with red, green left w red
        self.action_space = gym.spaces.Discrete(4)
        #self.action_space = gym.spaces.Box(4)

        # OBSERVATION SPACE cars in each direction
        #self.observation_space = gym.spaces.Discrete(36)
        self.observation_space = gym.spaces.Box(low=0, high=1, shape=(6,6))

        # CREATE GRID
        self.grid = np.zeros((6, 6))

        self.state = None
        self.viewer = None
        self.car_origin = dict()
        self.car_course = dict()
        self.car_x = dict()
        self.car_y = dict()

        self.car_list = dict()
        self.car_coor = dict()

        # GENERATE CARS

    def generate_car(self):
        global course
        global origin
        global x_coor
        global y_coor
        course = random.choice(courses_list)
        origin = random.choice(origin_list)
        if origin == 'North':
            x_coor = 0
            y_coor = 2
        elif origin == 'East':
            x_coor = 2
            y_coor = 5
        elif origin == 'South':
            x_coor = 5
            y_coor = 3
        elif origin == 'West':
            x_coor = 3
            y_coor = 0
        self.grid[x_coor, y_coor] = 1

    def add_car(self):
        self.car_origin[car_ID] = origin
        self.car_course[car_ID] = course
        self.car_x[car_ID] = x_coor
        self.car_y[car_ID] = y_coor

        self.car_list[car_ID] = course, origin, x_coor, y_coor
        self.car_coor[car_ID] = x_coor, y_coor

    def move_car(self, car_ID, course, origin, x_coor, y_coor, action):

        for cars in self.car_list:
            if self.car_list[cars] == {}:
                pass
            else:
                course = self.car_course[cars]
                origin = self.car_origin[cars]
                x_coor = self.car_x[cars]
                y_coor = self.car_y[cars]

                # SOUTH MOVING CONDITIONS
                if action == 1 and course == 'left' and origin == 'South':
                    if x_coor > 2:
                        self.car_coor[cars] = (x_coor-1,y_coor)
                        self.car_list[cars] = (course, origin, x_coor-1, y_coor)
                        self.car_x[cars] = x_coor - 1
                        hold = self.grid[x_coor - 1, y_coor]
                        self.grid[x_coor - 1, y_coor] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold
                    else:
                        if y_coor == 0:
                            self.grid[x_coor, y_coor] = 0
                            self.car_list[cars] = {}
                            self.car_coor[cars] = {}
                            self.car_x[cars] = {}
                            self.car_y[cars] = {}
                            self.car_origin[cars] = {}
                            self.car_course[cars] = {}
                        else:
                            self.car_coor[cars] = (x_coor, y_coor - 1)
                            self.car_list[cars] = (course, origin, x_coor, y_coor - 1)
                            self.car_y[cars] = y_coor - 1
                            hold = self.grid[x_coor, y_coor - 1]
                            self.grid[x_coor, y_coor] = self.grid[x_coor, y_coor - 1]
                            self.grid[x_coor, y_coor] = hold

                if action == 0 and course == 'straight' and origin == 'South':
                    if x_coor == 0:
                        self.grid[x_coor, y_coor] = 0
                        self.car_list[cars] = {}
                        self.car_coor[cars] = {}
                        self.car_x[cars] = {}
                        self.car_y[cars] = {}
                        self.car_origin[cars] = {}
                        self.car_course[cars] = {}
                    else:
                        self.car_coor[cars] = (x_coor - 1, y_coor)
                        self.car_list[cars] = (course, origin, x_coor - 1, y_coor)
                        self.car_x[cars] = x_coor - 1
                        hold = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = self.grid[x_coor - 1, y_coor]
                        self.grid[x_coor - 1, y_coor] = hold

                if action == 0 and course == 'right' and origin == 'South':
                    if x_coor > 3:
                        self.car_coor[cars] = (x_coor - 1, y_coor)
                        self.car_list[cars] = (course, origin, x_coor - 1, y_coor)
                        self.car_x[cars] = x_coor - 1
                        hold = self.grid[x_coor - 1, y_coor]
                        self.grid[x_coor-1, y_coor] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold
                    else:
                        if y_coor == 5:
                            self.grid[x_coor, y_coor] = 0
                            self.car_list[cars] = {}
                            self.car_coor[cars] = {}
                            self.car_x[cars] = {}
                            self.car_y[cars] = {}
                            self.car_origin[cars] = {}
                            self.car_course[cars] = {}
                        else:
                            self.car_coor[cars] = (x_coor, y_coor + 1)
                            self.car_list[cars] = (course, origin, x_coor, y_coor + 1)
                            self.car_y[cars] = y_coor + 1
                            hold = self.grid[x_coor, y_coor + 1]
                            self.grid[x_coor, y_coor+1] = self.grid[x_coor, y_coor]
                            self.grid[x_coor, y_coor] = hold

                # NORTH MOVING CONDITIONS
                if action == 1 and course == 'left' and origin == 'North':
                    if x_coor < 3:
                        self.car_coor[cars] = (x_coor+1,y_coor)
                        self.car_list[cars] = (course, origin, x_coor+1, y_coor)
                        self.car_x[cars] = x_coor + 1
                        hold = self.grid[x_coor + 1, y_coor]
                        self.grid[x_coor + 1, y_coor] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold
                    else:
                        if y_coor == 5:
                            self.grid[x_coor, y_coor] = 0
                            self.car_list[cars] = {}
                            self.car_coor[cars] = {}
                            self.car_x[cars] = {}
                            self.car_y[cars] = {}
                            self.car_origin[cars] = {}
                            self.car_course[cars] = {}
                        else:
                            self.car_coor[cars] = (x_coor, y_coor + 1)
                            self.car_list[cars] = (course, origin, x_coor, y_coor + 1)
                            self.car_y[cars] = y_coor + 1
                            hold = self.grid[x_coor, y_coor + 1]
                            self.grid[x_coor, y_coor + 1] = self.grid[x_coor, y_coor]
                            self.grid[x_coor, y_coor] = hold

                if action == 0 and course == 'straight' and origin == 'North':
                    if x_coor == 5:
                        self.grid[x_coor, y_coor] = 0
                        self.car_list[cars] = {}
                        self.car_coor[cars] = {}
                        self.car_x[cars] = {}
                        self.car_y[cars] = {}
                        self.car_origin[cars] = {}
                        self.car_course[cars] = {}
                    else:
                        self.car_coor[cars] = (x_coor + 1, y_coor)
                        self.car_list[cars] = (course, origin, x_coor + 1, y_coor)
                        self.car_x[cars] = x_coor + 1
                        hold = self.grid[x_coor + 1, y_coor]
                        self.grid[x_coor + 1, y_coor] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold

                if action == 0 and course == 'right' and origin == 'North':
                    if x_coor < 2:
                        self.car_coor[cars] = (x_coor + 1, y_coor)
                        self.car_list[cars] = (course, origin, x_coor + 1, y_coor)
                        self.car_x[cars] = x_coor + 1
                        hold = self.grid[x_coor + 1, y_coor]
                        self.grid[x_coor + 1, y_coor] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold
                    else:
                        if y_coor == 0:
                            self.grid[x_coor, y_coor] = 0
                            self.car_list[cars] = {}
                            self.car_coor[cars] = {}
                            self.car_x[cars] = {}
                            self.car_y[cars] = {}
                            self.car_origin[cars] = {}
                            self.car_course[cars] = {}
                        else:
                            self.car_coor[cars] = (x_coor, y_coor - 1)
                            self.car_list[cars] = (course, origin, x_coor, y_coor - 1)
                            self.car_y[cars] = y_coor - 1
                            hold = self.grid[x_coor, y_coor - 1]
                            self.grid[x_coor, y_coor - 1] = self.grid[x_coor, y_coor]
                            self.grid[x_coor, y_coor] = hold

                # EAST MOVING CONDITIONS
                if action == 3 and course == 'left' and origin == 'East':
                    if y_coor > 2:
                        self.car_coor[cars] = (x_coor, y_coor - 1)
                        self.car_list[cars] = (course, origin, x_coor, y_coor - 1)
                        self.car_y[cars] = y_coor - 1
                        hold = self.grid[2, y_coor - 1]
                        self.grid[x_coor, y_coor - 1] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold
                    else:
                        if x_coor == 5:
                            self.grid[x_coor, y_coor] = 0
                            self.car_list[cars] = {}
                            self.car_coor[cars] = {}
                            self.car_x[cars] = {}
                            self.car_y[cars] = {}
                            self.car_origin[cars] = {}
                            self.car_course[cars] = {}
                        else:
                            self.car_coor[cars] = (x_coor + 1, y_coor)
                            self.car_list[cars] = (course, origin, x_coor + 1, y_coor)
                            self.car_x[cars] = x_coor + 1
                            hold = self.grid[x_coor + 1, y_coor]
                            self.grid[x_coor + 1, y_coor] = self.grid[x_coor, y_coor]
                            self.grid[x_coor, y_coor] = hold

                if action == 2 and course == 'straight' and origin == 'East':
                    if y_coor == 0:
                        self.grid[x_coor, y_coor] = 0
                        self.car_list[cars] = {}
                        self.car_coor[cars] = {}
                        self.car_x[cars] = {}
                        self.car_y[cars] = {}
                        self.car_origin[cars] = {}
                        self.car_course[cars] = {}
                    else:
                        self.car_coor[cars] = (x_coor, y_coor - 1)
                        self.car_list[cars] = (course, origin, x_coor, y_coor - 1)
                        self.car_y[cars] = y_coor - 1
                        hold = self.grid[x_coor, y_coor - 1]
                        self.grid[x_coor, y_coor - 1] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold

                if action == 2 and course == 'right' and origin == 'East':
                    if y_coor > 3:
                        self.car_coor[cars] = (x_coor, y_coor - 1)
                        self.car_list[cars] = (course, origin, x_coor, y_coor - 1)
                        self.car_y[cars] = y_coor - 1
                        hold = self.grid[x_coor, y_coor - 1]
                        self.grid[x_coor, y_coor - 1] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold
                    else:
                        if x_coor == 0:
                            self.grid[x_coor, y_coor] = 0
                            self.car_list[cars] = {}
                            self.car_coor[cars] = {}
                            self.car_x[cars] = {}
                            self.car_y[cars] = {}
                            self.car_origin[cars] = {}
                            self.car_course[cars] = {}
                        else:
                            self.car_coor[cars] = (x_coor - 1, y_coor)
                            self.car_list[cars] = (course, origin, x_coor - 1, y_coor)
                            self.car_x[cars] = x_coor - 1
                            hold = self.grid[x_coor - 1, y_coor]
                            self.grid[x_coor - 1, y_coor] = self.grid[x_coor, y_coor]
                            self.grid[x_coor, y_coor] = hold

                # WEST MOVING CONDITIONS

                if action == 3 and course == 'left' and origin == 'West':
                    if y_coor < 3:
                        self.car_coor[cars] = (x_coor, y_coor + 1)
                        self.car_list[cars] = (course, origin, x_coor, y_coor + 1)
                        self.car_y[cars] = y_coor + 1
                        hold = self.grid[x_coor, y_coor + 1]
                        self.grid[x_coor, y_coor + 1] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold
                    else:
                        if x_coor == 0:
                            self.grid[x_coor, y_coor] = 0
                            self.car_list[cars] = {}
                            self.car_coor[cars] = {}
                            self.car_x[cars] = {}
                            self.car_y[cars] = {}
                            self.car_origin[cars] = {}
                            self.car_course[cars] = {}
                        else:
                            self.car_coor[cars] = (x_coor-1,y_coor)
                            self.car_list[cars] = (course, origin, x_coor-1, y_coor)
                            self.car_x[cars] = x_coor - 1
                            hold = self.grid[x_coor - 1, y_coor]
                            self.grid[x_coor - 1, y_coor] = self.grid[x_coor, y_coor]
                            self.grid[x_coor, y_coor] = hold

                if action == 2 and course == 'straight' and origin == 'West':
                    if y_coor == 5:
                        self.grid[x_coor, y_coor] = 0
                        self.car_list[cars] = {}
                        self.car_coor[cars] = {}
                        self.car_x[cars] = {}
                        self.car_y[cars] = {}
                        self.car_origin[cars] = {}
                        self.car_course[cars] = {}
                    else:
                        self.car_coor[cars] = (x_coor, y_coor + 1)
                        self.car_list[cars] = (course, origin, x_coor, y_coor + 1)
                        self.car_y[cars] = y_coor + 1
                        hold = self.grid[x_coor, y_coor + 1]
                        self.grid[x_coor, y_coor + 1] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold

                if action == 2 and course == 'right' and origin == 'West':
                    if y_coor < 2:
                        self.car_coor[cars] = (x_coor, y_coor + 1)
                        self.car_list[cars] = (course, origin, x_coor, y_coor + 1)
                        self.car_y[cars] = y_coor + 1
                        hold = self.grid[x_coor, y_coor + 1]
                        self.grid[x_coor, y_coor + 1] = self.grid[x_coor, y_coor]
                        self.grid[x_coor, y_coor] = hold
                    else:
                        if x_coor == 5:
                            self.grid[x_coor, y_coor] = 0
                            self.car_list[cars] = {}
                            self.car_coor[cars] = {}
                            self.car_x[cars] = {}
                            self.car_y[cars] = {}
                            self.car_origin[cars] = {}
                            self.car_course[cars] = {}
                        else:
                            self.car_coor[cars] = (x_coor + 1, y_coor)
                            self.car_list[cars] = (course, origin, x_coor + 1, y_coor)
                            self.car_x[cars] = x_coor + 1
                            hold = self.grid[x_coor + 1, y_coor]
                            self.grid[x_coor + 1, y_coor] = self.grid[x_coor, y_coor]
                            self.grid[x_coor, y_coor] = hold

        # REWARD FUNCTION
        global reward
        if self.grid[1,3] == 1:
            reward +=1
        if self.grid[2,1] == 1:
            reward +=1
        if self.grid[4,2] == 1:
            reward +=1
        if self.grid[3,4] == 1:
            reward +=1

        print("REWARD:", reward)

        global done, info

        # END RUN
        if len(self.car_coor) > 0:
            values = self.car_coor.values()
            list_values = list(values)
            for key in range(0, len(list_values)-1):
                for key2 in range(0, len(list_values)-1):
                    if key == key2:
                        break
                    else:
                        if list_values[key] == list_values[key2]:
                            done = True
                            print("Done is true", done)
                            break
                        else:
                            done = False
                            print("done is false", done)

        # INFO
        info = {}
        #print(done, "THIS IS WHAT IS BEING RETURNED")
        # RETURN
        return self.grid, reward, done, info

    def step(self, action):
        #Reset Reward
        global reward
        reward = 0

        #Generate Car
        global car_ID
        car_ID = car_ID + 1
        self.generate_car()

        #Generate Car Again If It Is a Repeat
        last1 = x_coor, y_coor
        values = self.car_coor.values()
        list_values = list(values)
        if (5,3) and (2,5) and (3,0) and (0,2) in list_values:
            pass
        else:
            while last1 in list_values:
                self.generate_car()
                last1 = x_coor, y_coor
            # Add New Car
            self.add_car()

        #Display Before and After Cars Moving, Move Car

        #print(self.grid, "GENERATE CAR")
        if done==False:
            self.move_car(car_ID, course, origin, x_coor, y_coor, action)
        #print(self.grid, "MOVE CAR")
        #print(self.move_car)
        #print(self.car_list, "CAR LIST")

        return self.grid, reward, done, info

    def reset(self):
        global done
        done = False #not working for maze RL

        # ACTIONS 4 are red light w green in other, red w green left, green with red, green left w red
        self.action_space = Discrete(4)

        # OBSERVATION SPACE cars in each direction
        self.observation_space = gym.spaces.Box(low=0, high=1, shape=(6, 6))

        # CREATE GRID
        self.grid = np.zeros((6, 6))

        self.car_list = dict()
        self.car_coor = dict()

        return self.grid

    def render(self, mode='human', close=False):
        screen_width = 600
        screen_height = 400

        world_width = 200
        scale = screen_width / world_width
        grid_width = 200
        grid_height = 200

        if self.viewer == None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(500, 500)
            self.viewer.set_bounds(-2.2, 2.2, -2.2, 2.2)
            '''
            rod = rendering.make_capsule(1, .2)
            rod.set_color(.8, .3, .3)
            self.pole_transform = rendering.Transform()
            rod.add_attr(self.pole_transform)
            self.viewer.add_geom(rod)
            axle = rendering.make_circle(.05)
            axle.set_color(0, 0, 0)
            self.viewer.add_geom(axle)
            '''
            a,b = -1.1,1.1
            grid = rendering.FilledPolygon([(a,b),(b,b),(b,a),(a,a)])
            self.env_transform = rendering.Transform()
            grid.add_attr(self.env_transform)
            self.viewer.add_geom(grid)

            c,d = -0.55,0.55
            road = rendering.FilledPolygon([(c,a),(d,a),()])

            return self.viewer.render(return_rgb_array=mode == "rgb_array")
        '''
          0 - green vertical, red horizontal
          1 - green left vertical, red horizontal
          2 - red vertical, green horizontal
          3 - red vertical, green left horizontal
        '''
