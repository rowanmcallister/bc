#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# Modified by Rowan McAllister on 20 April 2020

import argparse
import datetime
import glob
import os
import random
import sys
import time
from PIL import Image
from PIL.PngImagePlugin import PngInfo

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import math

from dotmap import DotMap

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue

from agents.navigation.agent import Agent, AgentState
from agents.navigation.local_planner import LocalPlanner
from agents.tools.misc import is_within_distance_ahead, compute_magnitude_angle


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

        self.start()

    def start(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False


def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))


class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds
        self.azimuth %= 360.0
        min_alt, max_alt = [20, 90]
        self.altitude = 0.5 * (max_alt + min_alt) + 0.5 * (max_alt - min_alt) * math.cos(self._t)

    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f)' % (self.altitude, self.azimuth)


class Storm(object):
    def __init__(self, precipitation):
        self._t = precipitation if precipitation > 0.0 else -50.0
        self._increasing = True
        self.clouds = 0.0
        self.rain = 0.0
        self.wetness = 0.0
        self.puddles = 0.0
        self.wind = 0.0
        self.fog = 0.0

    def tick(self, delta_seconds):
        delta = (1.3 if self._increasing else -1.3) * delta_seconds
        self._t = clamp(delta + self._t, -250.0, 100.0)
        self.clouds = clamp(self._t + 40.0, 0.0, 90.0)
        self.clouds = clamp(self._t + 40.0, 0.0, 60.0)
        self.rain = clamp(self._t, 0.0, 80.0)
        delay = -10.0 if self._increasing else 90.0
        self.puddles = clamp(self._t + delay, 0.0, 85.0)
        self.wetness = clamp(self._t * 5, 0.0, 100.0)
        self.wind = 5.0 if self.clouds <= 20 else 90 if self.clouds >= 70 else 40
        self.fog = clamp(self._t - 10, 0.0, 30.0)
        if self._t == -250.0:
            self._increasing = True
        if self._t == 100.0:
            self._increasing = False

    def __str__(self):
        return 'Storm(clouds=%d%%, rain=%d%%, wind=%d%%)' % (self.clouds, self.rain, self.wind)


class Weather(object):
    def __init__(self, world, changing_weather_speed):
        self.world = world
        self.reset()
        self.weather = world.get_weather()
        self.changing_weather_speed = changing_weather_speed
        self._sun = Sun(self.weather.sun_azimuth_angle, self.weather.sun_altitude_angle)
        self._storm = Storm(self.weather.precipitation)

    def reset(self):
        weather_params = carla.WeatherParameters(sun_altitude_angle=90.)
        self.world.set_weather(weather_params)

    def tick(self):
        self._sun.tick(self.changing_weather_speed)
        self._storm.tick(self.changing_weather_speed)
        self.weather.cloudiness = self._storm.clouds
        self.weather.precipitation = self._storm.rain
        self.weather.precipitation_deposits = self._storm.puddles
        self.weather.wind_intensity = self._storm.wind
        self.weather.fog_density = self._storm.fog
        self.weather.wetness = self._storm.wetness
        self.weather.sun_azimuth_angle = self._sun.azimuth
        self.weather.sun_altitude_angle = self._sun.altitude
        self.world.set_weather(self.weather)

    def __str__(self):
        return '%s %s' % (self._sun, self._storm)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vision_size', type=int, default=84)
    parser.add_argument('--vision_fov', type=int, default=90)
    parser.add_argument('--weather', default=False, action='store_true')
    parser.add_argument('--frame_skip', type=int, default=1),
    parser.add_argument('--steps', type=int, default=100000)
    parser.add_argument('--multiagent', default=False, action='store_true'),
    parser.add_argument('--lane', type=int, default=0)
    parser.add_argument('--lights', default=False, action='store_true')
    args = parser.parse_args()
    return args


class CarlaEnv(object):

    def __init__(self, args):
        self.render_display = True
        self.record_display = False
        self.record_vision = True
        self.record_dir = None
        self.vision_size = args.vision_size
        self.vision_fov = args.vision_fov
        self.changing_weather_speed = float(args.weather)
        self.frame_skip = args.frame_skip
        self.max_episode_steps = args.steps  # DMC uses this
        self.multiagent = args.multiagent
        self.start_lane = args.lane
        self.follow_traffic_lights = args.lights
        if self.record_display:
            assert self.render_display

        self.actor_list = []

        if self.render_display:
            pygame.init()
            self.render_display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
            self.font = get_font()
            self.clock = pygame.time.Clock()

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)

        self.world = self.client.get_world()
        self.map = self.world.get_map()

        # tests specific to map 4:
        if self.start_lane and self.map.name != "Town04":
            raise NotImplementedError

        # remove old vehicles and sensors (in case they survived)
        self.world.tick()
        actor_list = self.world.get_actors()
        for vehicle in actor_list.filter("*vehicle*"):
            print("Warning: removing old vehicle")
            vehicle.destroy()
        for sensor in actor_list.filter("*sensor*"):
            print("Warning: removing old sensor")
            sensor.destroy()

        self.vehicle = None
        self.vehicles_list = []  # their ids
        self.reset_vehicle()  # creates self.vehicle
        self.actor_list.append(self.vehicle)

        blueprint_library = self.world.get_blueprint_library()

        if self.render_display:
            self.camera_display = self.world.spawn_actor(
                blueprint_library.find('sensor.camera.rgb'),
                carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
                attach_to=self.vehicle)
            self.actor_list.append(self.camera_display)

        bp = blueprint_library.find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(self.vision_size))
        bp.set_attribute('image_size_y', str(self.vision_size))
        bp.set_attribute('fov', str(self.vision_fov))
        location = carla.Location(x=1.6, z=1.7)
        self.camera_vision = self.world.spawn_actor(bp, carla.Transform(location, carla.Rotation(yaw=0.0)), attach_to=self.vehicle)
        self.actor_list.append(self.camera_vision)

        if self.record_display or self.record_vision:
            if self.record_dir is None:
                self.record_dir = "carla-{}-{}x{}-fov{}".format(
                    self.map.name.lower(), self.vision_size, self.vision_size, self.vision_fov)
                if self.frame_skip > 1:
                    self.record_dir += '-{}'.format(self.frame_skip)
                if self.changing_weather_speed > 0.0:
                    self.record_dir += '-weather'
                if self.multiagent:
                    self.record_dir += '-mutiagent'
                if self.follow_traffic_lights:
                    self.record_dir += '-lights'
                self.record_dir += '-{}k'.format(self.max_episode_steps // 1000)

                now = datetime.datetime.now()
                self.record_dir += now.strftime("-%Y-%m-%d-%H-%M-%S")
            os.mkdir(self.record_dir)

        if self.render_display:
            self.sync_mode = CarlaSyncMode(self.world, self.camera_display, self.camera_vision, fps=20)
        else:
            self.sync_mode = CarlaSyncMode(self.world, self.camera_vision, fps=20)

        # weather
        self.weather = Weather(self.world, self.changing_weather_speed)

        # dummy variables, to match deep mind control's APIs
        low = -1.0
        high = 1.0
        self.action_space = DotMap()
        self.action_space.low.min = lambda: low
        self.action_space.high.max = lambda: high
        self.action_space.shape = [2]
        self.observation_space = DotMap()
        self.observation_space.shape = (3, self.vision_size, self.vision_size)
        self.observation_space.dtype = np.dtype(np.uint8)
        self.reward_range = None
        self.metadata = None
        self.action_space.sample = lambda: np.random.uniform(low=low, high=high, size=self.action_space.shape[0]).astype(np.float32)

        # roaming carla agent
        self.agent = None
        self.count = 0
        self.world.tick()
        self.reset()  # creates self.agent

    def reset(self):
        self.reset_vehicle()
        self.world.tick()
        self.reset_other_vehicles()
        self.world.tick()
        self.agent = RoamingAgent(self.vehicle, follow_traffic_lights=self.follow_traffic_lights)
        self.count = 0

        # get obs:
        obs, _, _, _ = self.step()
        return obs

    def reset_vehicle(self):

        if self.map.name == "Town04":
            start_lane = self.start_lane if self.start_lane is not None else np.random.choice([1, 2, 3, 4])  # their positive values, not negative
            start_x = 1.5 + 3.5 * start_lane
            vehicle_init_transform = carla.Transform(carla.Location(x=start_x, y=0, z=0.1), carla.Rotation(yaw=-90))
        else:
            init_transforms = self.world.get_map().get_spawn_points()
            vehicle_init_transform = random.choice(init_transforms)

        if self.vehicle is None:  # then create the ego vehicle
            blueprint_library = self.world.get_blueprint_library()
            vehicle_blueprint = blueprint_library.find('vehicle.audi.a2')
            self.vehicle = self.world.spawn_actor(vehicle_blueprint, vehicle_init_transform)

        self.vehicle.set_transform(vehicle_init_transform)
        self.vehicle.set_velocity(carla.Vector3D())
        self.vehicle.set_angular_velocity(carla.Vector3D())

    def reset_other_vehicles(self):
        if not self.multiagent:
            return

        # clear out old vehicles
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])
        self.world.tick()
        self.vehicles_list = []

        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        traffic_manager.set_synchronous_mode(True)
        blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]

        num_vehicles = 20
        if self.map.self.map.name == "Town04":
            road_id = 47
            road_length = 117.
            init_transforms = []
            for _ in range(num_vehicles):
                lane_id = random.choice([-1, -2, -3, -4])
                vehicle_s = np.random.uniform(road_length)  # length of road 47
                init_transforms.append(self.map.get_waypoint_xodr(road_id, lane_id, vehicle_s).transform)
        else:
            init_transforms = self.world.get_map().get_spawn_points()
            init_transforms = random.choice(init_transforms, num_vehicles)

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for transform in init_transforms:
            transform.location.z += 0.1  # otherwise can collide with the road it starts on
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetAutopilot(carla.command.FutureActor, True)))

        for response in self.client.apply_batch_sync(batch, False):
            self.vehicles_list.append(response.actor_id)

        for response in self.client.apply_batch_sync(batch):
            if response.error:
                pass
            else:
                self.vehicles_list.append(response.actor_id)

        traffic_manager.global_percentage_speed_difference(30.0)

    def compute_action(self):
        return self.agent.run_step()

    def step(self, action=None, traffic_light_color=""):
        rewards = []
        for _ in range(self.frame_skip):  # default 1
            next_obs, reward, done, info = self._simulator_step(action, traffic_light_color)
            rewards.append(reward)
            if done:
                break
        return next_obs, np.mean(rewards), done, info

    def _simulator_step(self, action, traffic_light_color):

        if self.render_display:
            if should_quit():
                return
            self.clock.tick()

        if action is None:
            throttle, steer, brake = 0., 0., 0.
        else:
            throttle, steer, brake = action.throttle, action.steer, action.brake
            vehicle_control = carla.VehicleControl(
                throttle=throttle,  # [0,1]
                steer=steer,  # [-1,1]
                brake=brake,  # [0,1]
                hand_brake=False,
                reverse=False,
                manual_gear_shift=False
            )
            self.vehicle.apply_control(vehicle_control)

        # Advance the simulation and wait for the data.
        if self.render_display:
            snapshot, display_image, vision_image = self.sync_mode.tick(timeout=2.0)
        else:
            snapshot, vision_image = self.sync_mode.tick(timeout=2.0)

        # Weather evolves
        self.weather.tick()

        # Draw the display.
        if self.render_display:
            draw_image(self.render_display, display_image)
            self.render_display.blit(self.font.render('Frame %d' % self.count, True, (255, 255, 255)), (8, 10))
            self.render_display.blit(self.font.render('Control: %5.2f thottle, %5.2f steer, %5.2f brake' % (throttle, steer, brake), True, (255, 255, 255)), (8, 28))
            self.render_display.blit(self.font.render('Traffic light: ' + traffic_light_color, True, (255, 255, 255)), (8, 46))
            self.render_display.blit(self.font.render(str(self.weather), True, (255, 255, 255)), (8, 64))
            pygame.display.flip()

        # Format rl image
        bgra = np.array(vision_image.raw_data).reshape(self.vision_size, self.vision_size, 4)  # BGRA format
        bgr = bgra[:, :, :3]  # BGR format (84 x 84 x 3)
        rgb = np.flip(bgr, axis=2)  # RGB format (84 x 84 x 3)

        if self.render_display and self.record_display:
            image_name = os.path.join(self.record_dir, "display%08d.jpg" % self.count)
            pygame.image.save(self.render_display, image_name)
            # # Can animate with:
            # ffmpeg -r 20 -pattern_type glob -i 'display*.jpg' carla.mp4
        if self.record_vision:
            image_name = os.path.join(self.record_dir, "vision%08d.png" % self.count)
            im = Image.fromarray(rgb)

            # add any meta data you like into the image before we save it:
            metadata = PngInfo()
            metadata.add_text("throttle", str(throttle))
            metadata.add_text("steer", str(steer))
            metadata.add_text("brake", str(brake))
            metadata.add_text("lights", traffic_light_color)
            im.save(image_name, "PNG", pnginfo=metadata)

            # # To read these images later, you can run something like this:
            # from PIL.PngImagePlugin import PngImageFile
            # im = PngImageFile("vision00001234.png")
            # throttle = float(im.text['throttle'])  # range [0, 1]
            # steer = float(im.text['steer'])  # range [-1, 1]
            # brake = float(im.text['brake'])  # range [0, 1]
            # lights = im.text['lights']  # traffic lights color, [NONE, JUNCTION, RED, YELLOW, GREEN]
        self.count += 1

        next_obs = rgb  # 84 x 84 x 3
        # # To inspect images, run:
        # import pdb; pdb.set_trace()
        # import matplotlib.pyplot as plt
        # plt.imshow(next_obs)
        # plt.show()

        done = self.count >= self.max_episode_steps
        if done:
            print("Episode success: I've reached the episode horizon ({}).".format(self.max_episode_steps))
        reward = 0.  # define yourself
        info = None
        return next_obs, reward, done, info

    def finish(self):
        print('destroying actors.')
        for actor in self.actor_list:
            actor.destroy()
        print('\ndestroying %d vehicles' % len(self.vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])
        time.sleep(0.5)
        pygame.quit()
        print('done.')


class LocalPlannerModified(LocalPlanner):

    def __del__(self):
        pass  # otherwise it deletes our vehicle object

    def run_step(self):
        return super().run_step(debug=False)  # otherwise by default shows waypoints, that interfere with our camera


class RoamingAgent(Agent):
    """
    RoamingAgent implements a basic agent that navigates scenes making random
    choices when facing an intersection.

    This agent respects traffic lights and other vehicles.
    """

    def __init__(self, vehicle, follow_traffic_lights=True):
        """

        :param vehicle: actor to apply to local planner logic onto
        """
        super(RoamingAgent, self).__init__(vehicle)
        self._proximity_threshold = 10.0  # meters
        self._state = AgentState.NAVIGATING
        self._local_planner = LocalPlannerModified(self._vehicle)
        self._follow_traffic_lights = follow_traffic_lights

    def run_step(self):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """

        # is there an obstacle in front of us?
        hazard_detected = False

        # retrieve relevant elements for safe navigation, i.e.: traffic lights and other vehicles
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")

        # check possible obstacles
        vehicle_state, vehicle = self._is_vehicle_hazard(vehicle_list)
        if vehicle_state:

            self._state = AgentState.BLOCKED_BY_VEHICLE
            hazard_detected = True

        # check for the state of the traffic lights
        traffic_light_color = self._is_light_red(lights_list)
        if traffic_light_color == 'RED' and self._follow_traffic_lights:
            self._state = AgentState.BLOCKED_RED_LIGHT
            hazard_detected = True

        if hazard_detected:
            control = self.emergency_stop()
        else:
            self._state = AgentState.NAVIGATING
            # standard local planner behavior
            control = self._local_planner.run_step()

        return control, traffic_light_color

    # override case class
    def _is_light_red_europe_style(self, lights_list):
        """
        This method is specialized to check European style traffic lights.
        Only suitable for Towns 03 -- 07.
        """
        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        traffic_light_color = "NONE"  # default, if no traffic lights are seen

        for traffic_light in lights_list:
            object_waypoint = self._map.get_waypoint(traffic_light.get_location())
            if object_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                    object_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            if is_within_distance_ahead(traffic_light.get_transform(),
                                        self._vehicle.get_transform(),
                                        self._proximity_threshold):
                if traffic_light.state == carla.TrafficLightState.Red:
                    return "RED"
                elif traffic_light.state == carla.TrafficLightState.Yellow:
                    traffic_light_color = "YELLOW"
                elif traffic_light.state == carla.TrafficLightState.Green:
                    if traffic_light_color is not "YELLOW":  # (more severe)
                        traffic_light_color = "GREEN"
                else:
                    import pdb; pdb.set_trace()
                    # investigate https://carla.readthedocs.io/en/latest/python_api/#carlatrafficlightstate

        return traffic_light_color

    # override case class
    def _is_light_red_us_style(self, lights_list, debug=False):
        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        traffic_light_color = "NONE"  # default, if no traffic lights are seen

        if ego_vehicle_waypoint.is_junction:
            # It is too late. Do not block the intersection! Keep going!
            return "JUNCTION"

        if self._local_planner.target_waypoint is not None:
            if self._local_planner.target_waypoint.is_junction:
                min_angle = 180.0
                sel_magnitude = 0.0
                sel_traffic_light = None
                for traffic_light in lights_list:
                    loc = traffic_light.get_location()
                    magnitude, angle = compute_magnitude_angle(loc,
                                                               ego_vehicle_location,
                                                               self._vehicle.get_transform().rotation.yaw)
                    if magnitude < 60.0 and angle < min(25.0, min_angle):
                        sel_magnitude = magnitude
                        sel_traffic_light = traffic_light
                        min_angle = angle

                if sel_traffic_light is not None:
                    if debug:
                        print('=== Magnitude = {} | Angle = {} | ID = {}'.format(
                            sel_magnitude, min_angle, sel_traffic_light.id))

                    if self._last_traffic_light is None:
                        self._last_traffic_light = sel_traffic_light

                    if self._last_traffic_light.state == carla.TrafficLightState.Red:
                        return "RED"
                    elif self._last_traffic_light.state == carla.TrafficLightState.Yellow:
                        traffic_light_color = "YELLOW"
                    elif self._last_traffic_light.state == carla.TrafficLightState.Green:
                        if traffic_light_color is not "YELLOW":  # (more severe)
                            traffic_light_color = "GREEN"
                    else:
                        import pdb; pdb.set_trace()
                        # investigate https://carla.readthedocs.io/en/latest/python_api/#carlatrafficlightstate
                else:
                    self._last_traffic_light = None

        return traffic_light_color


if __name__ == '__main__':

    # example call:
    # ./PythonAPI/util/config.py --map Town01 --delta-seconds 0.05
    # python PythonAPI/carla/agents/navigation/data_collection_agent.py --vision_size 256 --vision_fov 90 --steps 10000 --weather --lights

    args = parse_args()
    env = CarlaEnv(args)

    try:
        done = False
        while not done:
            action, traffic_light_color = env.compute_action()
            next_obs, reward, done, info = env.step(action, traffic_light_color)
    finally:
        env.finish()
