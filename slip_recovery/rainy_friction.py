#!/usr/bin/env python3

import carla
import random
import time
import numpy as np
import sys
import pygame
import datetime
import os
import math
import collections
from typing import Deque

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim, flags=pygame.SRCALPHA)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim, flags=pygame.SRCALPHA)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        alpha = int(min(255, 500.0 * self.seconds_left))
        self.surface.set_alpha(alpha)

    def render(self, display):
        display.blit(self.surface, self.pos)


class HelpText(object):
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim, flags=pygame.SRCALPHA)
        self.surface.fill((0, 0, 0, 0))
        for i, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, i * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else (fonts[0] if fonts else pygame.font.get_default_font())
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        try:
            self.frame = timestamp.frame_count
            self.simulation_time = timestamp.elapsed_seconds
        except Exception:
            pass

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        transform = getattr(world, 'player').get_transform()
        vel = getattr(world, 'player').get_velocity()
        control = getattr(world, 'player').get_control()
        heading = ''
        if abs(transform.rotation.yaw) < 89.5:
            heading += 'N'
        if abs(transform.rotation.yaw) > 90.5:
            heading += 'S'
        if 179.5 > transform.rotation.yaw > 0.5:
            heading += 'E'
        if -0.5 > transform.rotation.yaw > -179.5:
            heading += 'W'

        colhist = world.collision_sensor.get_collision_history()
        if len(colhist) < 200:
            collision = [0.0] * (200 - len(colhist)) + list(colhist)
        else:
            collision = colhist[-200:]
        max_col = max(1.0, max(collision))
        collision_norm = [x / max_col for x in collision]

        vehicles = world.get_actors().filter('vehicle.*')

        speed_kmh = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.get_map().name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % speed_kmh,
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (transform.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (transform.location.x, transform.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon if hasattr(world.gnss_sensor, 'lon') else 0.0)),
            'Height:  % 18.0f m' % transform.location.z,
            ''
        ]

        if isinstance(control, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', control.throttle, 0.0, 1.0),
                ('Steer:', control.steer, -1.0, 1.0),
                ('Brake:', control.brake, 0.0, 1.0),
                ('Reverse:', control.reverse),
                ('Hand brake:', control.hand_brake),
                ('Manual:', control.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(control.gear, control.gear)
            ]
        elif isinstance(control, carla.WalkerControl):
            self._info_text += [
                ('Speed:', control.speed, 0.0, 5.556),
                ('Jump:', control.jump)
            ]

        self._info_text += [
            '',
            'Collision:',
            collision_norm,
            '',
            'Number of vehicles: % 8d' % len(vehicles)
        ]

        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']

        def dist(l):
            return math.sqrt((l.x - transform.location.x)**2 + (l.y - transform.location.y)**2 + (l.z - transform.location.z)**2)
        vehicles = [(dist(x.get_location()), x) for x in vehicles if x.id != world.player.id]

        for dist_v, veh in sorted(vehicles):
            if dist_v > 200.0:
                break
            vehicle_type = get_actor_display_name(veh, truncate=22)
            self._info_text.append('% 4dm %s' % (dist_v, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((320, self.dim[1]), flags=pygame.SRCALPHA)
            info_surface.set_alpha(160)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 160
            bar_width = 150
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        fig = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + fig * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (fig * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)

# ---------- Sensors wrappers ----------

class CollisionSensor(object):
    def __init__(self, world, vehicle, role_name="slip_test_collision"):
        self._history: Deque[float] = collections.deque(maxlen=200)
        for _ in range(200):
            self._history.append(0.0)
        bp = world.get_blueprint_library().find('sensor.other.collision')
        bp.set_attribute('role_name', role_name)
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
        self.sensor.listen(lambda event: self._on_collision(event))

    def _on_collision(self, event):
        self._history.append(1.0)

    def get_collision_history(self):
        return list(self._history)

    def stop(self):
        try:
            self.sensor.stop()
        except Exception:
            pass
    def destroy(self):
        try:
            self.sensor.destroy()
        except Exception:
            pass

class GnssSensor(object):
    def __init__(self, world, vehicle, role_name="slip_test_gnss"):
        self.lat = 0.0
        self.lon = 0.0
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        bp.set_attribute('role_name', role_name)
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
        self.sensor.listen(lambda event: self._on_gnss(event))

    def _on_gnss(self, event):
        self.lat = event.latitude
        self.lon = event.longitude

    def stop(self):
        try:
            self.sensor.stop()
        except Exception:
            pass
    def destroy(self):
        try:
            self.sensor.destroy()
        except Exception:
            pass

# ---------- Image processing / camera callback ----------

def process_image_to_rgb(image, out_dict):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    try:
        array = array.reshape((image.height, image.width, 4))
        bgr = array[:, :, :3]
        # Convert BGR -> RGB
        rgb = bgr[:, :, ::-1]
    except Exception:
        # If reshape fails, fallback to generic conversion
        rgb = np.copy(np.frombuffer(image.raw_data, dtype=np.uint8)).reshape((image.height, image.width, 3))
    out_dict['frame'] = rgb

def numpy_to_pygame_surface(frame_numpy):
    surf = pygame.surfarray.make_surface(frame_numpy.swapaxes(0, 1))
    return surf

# ---------- helpers for wheel angular velocities ----------

def get_wheel_angular_velocities(vehicle):
    try:
        omegas = vehicle.get_wheel_angular_velocity()
        return list(omegas)
    except Exception:
        try:
            wc = vehicle.get_physics_control().wheels
            return [0.0] * len(wc)
        except Exception:
            return []

# ---------- Weather & friction helper ----------

def set_weather(world):
    weather = carla.WeatherParameters(
        cloudiness=10.0,
        precipitation=30.0,
        precipitation_deposits=30.0,
        wind_intensity=10.0,
        sun_azimuth_angle=70.0,
        sun_altitude_angle=70.0,
        wetness=40.0
    )
    world.set_weather(weather)
    return weather

def apply_weather_based_friction(vehicle, wetness, precipitation_deposits):
    physics_control = vehicle.get_physics_control()
    wheels = physics_control.wheels
    friction_scale = 1.0 - (wetness + precipitation_deposits) / 200.0
    for w in wheels:
        w.tire_friction = max(0.2, w.tire_friction * friction_scale)
    physics_control.wheels = wheels
    vehicle.apply_physics_control(physics_control)
    pc = vehicle.get_physics_control()
    applied = [round(w.tire_friction, 3) for w in pc.wheels]
    print("Applied wheel frictions:", applied)

# ---------- Main pygame display loop ----------

def main_pygame_display_loop(world, vehicle, image_data_dict, hud, run_time=20.0):
    pygame.init()
    width, height = 800, 600
    display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("CARLA - Downward Camera + HUD")
    clock = pygame.time.Clock()

    running = True
    start_time = time.time()
    try:
        while running and (time.time() - start_time) < run_time:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        running = False
                    elif event.key == pygame.K_h:
                        hud.help.toggle()
                    elif event.key == pygame.K_i:
                        hud.toggle_info()

            # draw latest camera frame
            frame = image_data_dict.get('frame', None)
            if frame is not None:
                surf = numpy_to_pygame_surface(frame)
                surf = pygame.transform.smoothscale(surf, (width, height))
                display.blit(surf, (0, 0))
            else:
                display.fill((0, 0, 0))

            # telemetry
            v = vehicle.get_velocity()
            veh_speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
            yaw_rate = vehicle.get_angular_velocity().z
            wheel_omegas = get_wheel_angular_velocities(vehicle)
            ctrl = vehicle.get_control()
            collision_flag = any([x > 0.5 for x in world.collision_sensor.get_collision_history()[-20:]])

            # Build custom info lines (insert at top of HUD info)
            tnow = datetime.timedelta(seconds=int(time.time() - start_time))
            extra_lines = [
                f"Elapsed: {tnow}",
                f"Lin vel: {veh_speed:.2f} m/s",
                f"Yaw rate: {yaw_rate:.3f} rad/s",
                f"Wheels omega: {', '.join(f'{om:.2f}' for om in wheel_omegas)}",
                f"Control: throttle={ctrl.throttle:.3f} steer={ctrl.steer:.3f} brake={ctrl.brake:.3f}",
                f"Collision recent: {collision_flag}"
            ]

            hud.tick(world, clock)

            for i, line in enumerate(reversed(extra_lines)):
                hud._info_text.insert(0, line)

            hud.render(display)

            # pop the extra lines we inserted so they don't accumulate
            for _ in extra_lines:
                try:
                    hud._info_text.pop(0)
                except Exception:
                    pass

            pygame.display.flip()
            clock.tick(30)
    finally:
        pygame.quit()

# ---------- main flow ----------

def destroy_actor_safe(actor):
    try:
        if actor is not None:
            actor.destroy()
    except Exception:
        pass

def destroy_all_actors(world):
    print("Destroying leftover actors from previous runs...")
    actors = world.get_actors()
    for actor in actors:
        try:
            if 'vehicle' in actor.type_id or 'sensor' in actor.type_id:
                actor.destroy()
        except Exception:
            pass
    print("All previous actors destroyed.")

def main():
    pygame.init()
    pygame.font.init()

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)

    world_obj = client.get_world()
    if 'Town04' not in world_obj.get_map().name:
        world_obj = client.load_world('Town04')

    random.seed(42)

    weather = set_weather(world_obj)
    wetness = weather.wetness
    precipitation_deposits = weather.precipitation_deposits

    bp_lib = world_obj.get_blueprint_library()
    try:
        vehicle_bp = bp_lib.find('vehicle.tesla.model3')
        if vehicle_bp is None:
            raise Exception()
    except Exception:
        vehicle_bp = bp_lib.filter('vehicle.*')[0]

    spawn_points = world_obj.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found in the current map!")
    spawn_point = spawn_points[62 % len(spawn_points)]

    actor_list = []
    collision_sensor = None
    gnss_sensor = None
    camera = None
    try:
        vehicle = world_obj.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)
        print("Spawned vehicle:", vehicle.type_id)

        apply_weather_based_friction(vehicle, wetness, precipitation_deposits)

        # attach sensors
        gnss_sensor = GnssSensor(world_obj, vehicle)
        actor_list.append(gnss_sensor.sensor)
        collision_sensor = CollisionSensor(world_obj, vehicle)
        actor_list.append(collision_sensor.sensor)

        # attach downward camera (relative transform above vehicle)
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '110')
        camera_bp.set_attribute('role_name', 'slip_test_camera')

        camera_transform = carla.Transform(carla.Location(x=0.0, z=25.0), carla.Rotation(pitch=-90.0))
        camera = world_obj.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        actor_list.append(camera)
        print("Attached downward-facing RGB camera")

        image_data = {'frame': None}
        camera.listen(lambda image: process_image_to_rgb(image, image_data))

        world_obj.player = vehicle
        world_obj.gnss_sensor = gnss_sensor
        world_obj.collision_sensor = collision_sensor

        hud = HUD(800, 600)

        # start driving
        vehicle.set_autopilot(False)
        vehicle.apply_control(carla.VehicleControl(throttle=0.7))
        print("Vehicle accelerating...")

        main_pygame_display_loop(world_obj, vehicle, image_data, hud, run_time=18.0)

        # perform a controlled turn
        vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.6))
        print("Vehicle turning...")
        main_pygame_display_loop(world_obj, vehicle, image_data, hud, run_time=6.0)

        # braking
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        print("Braking...")
        time.sleep(2.0)

    except Exception as e:
        print("Error occurred:", e)
        import traceback
        traceback.print_exc()

    finally:
        print("Cleaning up...")

        try:
            if camera is not None:
                camera.stop()
        except Exception:
            pass

        try:
            if gnss_sensor is not None:
                gnss_sensor.stop()
        except Exception:
            pass

        try:
            if collision_sensor is not None:
                collision_sensor.stop()
        except Exception:
            pass

        # destroy spawned actors
        for a in actor_list:
            destroy_actor_safe(a)

        try:
            if collision_sensor is not None:
                collision_sensor.destroy()
        except Exception:
            pass
        try:
            if gnss_sensor is not None:
                gnss_sensor.destroy()
        except Exception:
            pass

        time.sleep(0.5)
        print("Cleanup complete.")

if __name__ == "__main__":
    main()
