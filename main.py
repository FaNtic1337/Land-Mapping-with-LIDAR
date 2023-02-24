import pygame, math, sys


# SUPPORT FUNCTIONS
def round_to_descrete(num):
    # if num > 0 - round to up; if num < 0 - round to down
    if num > 0:
        num = math.ceil(num)
    else:
        num = math.floor(num)
    return num


class Robot:
    def __init__(self, global_center, tilesize, lidar_scan_zone):
        # DEFINE VARIABLES
        self.screen = pygame.display.get_surface()
        self.font = pygame.font.Font(None, 30)

        self.global_center = global_center  # the center relative to which the robot was localized
        self.tilesize = tilesize  # size of cell on map in centimeters
        self.lidar_scan_zone = lidar_scan_zone  # scan zone in degrees

        self.trajectory = []
        self.obstacles = []
        self.step = 0

        self.odometry, self.lidar_data, self.scans = self.parse_and_format_log('examp5.txt')

    # SUPPORT FUNCTIONS
    def draw_tile(self, tile_pos, color):
        rect = pygame.Rect(tile_pos[0] * self.tilesize, tile_pos[1] * self.tilesize, self.tilesize, self.tilesize)
        pygame.draw.rect(self.screen, color, rect)

    def debug(self, info, x=10, y=10):

        debug_surf = self.font.render(str(info), True, 'White')
        debug_rect = debug_surf.get_rect(topleft=(x, y))
        pygame.draw.rect(self.screen, 'Black', debug_rect)
        self.screen.blit(debug_surf, debug_rect)

    #  DATA FUNCTIONS
    def parse_and_format_log(self, file_name):
        # Parsing data
        original_robot_data = []
        original_lidar_data = []

        scans = 0

        with open(file_name) as log:
            for line in log:
                # Parsing robot data - global (x and y in meters, φ in radans)
                original_robot_data.append(list(map(float, line[:line.find(";")].split(','))))
                # Parsing lidar data - list of distances to obstacles in meters
                original_lidar_data.append(list(map(float, line[line.find(";") + 1:-1].split(','))))

                # Counting amount of scans
                scans += 1

        # Formating data
        formated_robot_data = []
        formated_lidar_data = []

        # Robot
        for data in original_robot_data:

            # For robot_x and robot_x: Meters -> Сentimeters -> Descretes;
            robot_x = self.global_center[0] + data[0] * 100 / self.tilesize
            robot_y = self.global_center[1] - data[1] * 100 / self.tilesize

            # For robot_angle: Radans -> Degrees (from 0 to 360 system)
            if data[2] < 0:
                robot_angle = math.degrees(data[2]) + 360
            else:
                robot_angle = math.degrees(data[2])

            formated_robot_data.append((round_to_descrete(robot_x), round_to_descrete(robot_y), robot_angle))

        # Obstacles
        for counter, distances in enumerate(original_lidar_data):

            detected_objects = len(distances)
            resolution_angle = self.lidar_scan_zone / detected_objects
            robot_angle = formated_robot_data[counter][2]

            obstacles_per_scan = []

            for i in range(detected_objects):
                if (distances[i] > 0.5) and (distances[i] < 5):
                    # Current angle to get static obstacles
                    angle = math.radians(-robot_angle + resolution_angle * i - self.lidar_scan_zone / 2)

                    # For lidar_distances and robot_angle: Meters + radians -> obstacle_x and obstacle_y in descretes;
                    obstacle_x = formated_robot_data[counter][0] + math.cos(angle) * distances[i] * 100 / self.tilesize
                    obstacle_y = formated_robot_data[counter][1] + math.sin(angle) * distances[i] * 100 / self.tilesize

                    obstacle_pos = (round_to_descrete(obstacle_x), round_to_descrete(obstacle_y))
                    obstacles_per_scan.append(obstacle_pos)

            formated_lidar_data.append(obstacles_per_scan)

        return formated_robot_data, formated_lidar_data, scans

    # GRAPHIC FUNCTIONS
    def draw_background(self):
        rows = self.screen.get_height() // self.tilesize
        cols = self.screen.get_width() // self.tilesize

        self.screen.fill((176, 196, 222))

        for i in range(rows):
            if i % 2 == 0:
                for j in range(0, cols, 2):
                    self.draw_tile((j, i), (119, 136, 153))
            else:
                for j in range(1, cols, 2):
                    self.draw_tile((j, i), (119, 136, 153))

        # (0, 0) point or global center
        self.draw_tile(self.global_center, 'red')

    def draw_robot(self, counter):
        robot_x = self.odometry[counter][0]
        robot_y = self.odometry[counter][1]
        robot_angle = self.odometry[counter][2]

        robot_pos = (robot_x, robot_y)

        if robot_pos not in self.trajectory:
            self.trajectory.append(robot_pos)

        self.draw_tile(robot_pos, 'green')

        self.debug("Robot_x: " + str(robot_x))
        self.debug("Robot_y: " + str(robot_y), y=40)
        self.debug("Robot direction: " + str(robot_angle), y=70)
        self.debug("Step: " + str(self.step), y=100)

    def draw_obstacle(self, counter):
        for obstacle_pos in self.lidar_data[counter]:

            if obstacle_pos not in self.obstacles:
                self.obstacles.append(obstacle_pos)

            self.draw_tile(obstacle_pos, 'blue')

    def draw_trajectory(self):
        for i in range(len(self.trajectory)):
            self.draw_tile(self.trajectory[i], 'dark green')

    def draw_map(self):
        for i in range(len(self.obstacles)):
            self.draw_tile(self.obstacles[i], '#000080')

    # CONTROLS FUNCTIONS
    def check_inputs(self):
        keys = pygame.key.get_pressed()

        if keys[pygame.K_RIGHT]:
            if self.step < self.scans - 1:
                self.step += 1
            else:
                self.step = self.scans - 1
        elif keys[pygame.K_LEFT]:
            if self.step > 1:
                self.step -= 1
            else:
                self.step = 0

    def run(self):

        self.draw_background()
        self.draw_map()
        self.draw_obstacle(self.step)
        self.draw_trajectory()
        self.draw_robot(self.step)

        self.check_inputs()


class App:
    def __init__(self, map_size, tilesize, FPS):

        self.map_size = map_size  # how much tiles
        self.tilesize = tilesize  # in centimeters
        self.FPS = FPS

        # SETUP
        pygame.init()
        self.screen = pygame.display.set_mode(self.get_resolution())
        pygame.display.set_caption('Land Mapping with LIDAR')
        self.clock = pygame.time.Clock()

        # Creating an instance of Robot
        self.robot_with_lidar = Robot(global_center=(25, 25), tilesize=self.tilesize, lidar_scan_zone=240)

    def get_resolution(self):

        resolution = (self.map_size[0] * self.tilesize, self.map_size[1] * self.tilesize)

        return resolution

    def run(self):

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self.robot_with_lidar.run()

            pygame.display.update()
            self.clock.tick(self.FPS)


if __name__ == '__main__':
    app = App(map_size=(150, 100), tilesize=10, FPS=10)
    app.run()
