import carla
import math
import time
import random

def set_weather(world):
    weather = carla.WeatherParameters(
        cloudiness=80.0,
        precipitation=70.0,
        precipitation_deposits=60.0,
        wetness=80.0,
        wind_intensity=20.0,
        sun_altitude_angle=70.0,
        fog_density=10.0
    )
    world.set_weather(weather)
    return weather

def compute_measured_friction(original_friction, wetness, precipitation_deposits):
    measured_friction = (
        math.exp(-0.916 * wetness / 100.0)
        * (1 - wetness / 100.0)**3 * 0.6
        + 0.4
        - 0.1 * precipitation_deposits / 100.0
    )
    return original_friction * measured_friction

def apply_weather_based_friction(vehicle, wetness, precipitation_deposits):
    physics_control = vehicle.get_physics_control()

    new_wheels = []
    for wheel in physics_control.wheels:
        old_friction = wheel.tire_friction
        new_friction = compute_measured_friction(old_friction, wetness, precipitation_deposits)
        new_wheel = carla.WheelPhysicsControl(
            tire_friction=new_friction,
            damping_rate=wheel.damping_rate,
            max_steer_angle=wheel.max_steer_angle,
            radius=wheel.radius,
            max_brake_torque=wheel.max_brake_torque,
            max_handbrake_torque=wheel.max_handbrake_torque
        )
        new_wheels.append(new_wheel)

    physics_control.wheels = new_wheels
    vehicle.apply_physics_control(physics_control)

    # Verify the applied values
    applied_pc = vehicle.get_physics_control()
    for i, w in enumerate(applied_pc.wheels):
        print(f"Wheel {i}: tire_friction = {w.tire_friction:.3f}")

    # Wait a moment to let the physics engine update
    time.sleep(0.5)


def move_spectator_to_vehicle(world, vehicle):
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    # place spectator 30 meters above car looking straight down
    top_down_transform = carla.Transform(
        carla.Location(x=transform.location.x, y=transform.location.y, z=transform.location.z + 30),
        carla.Rotation(pitch=-90)
    )
    spectator.set_transform(top_down_transform)

if __name__ == "__main__":
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    random.seed(42)

    weather = set_weather(world)
    wetness = weather.wetness
    precipitation_deposits = weather.precipitation_deposits

    # get a valid spawn point
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found in the current map!")
    spawn_point = spawn_points[0]

    # try multiple vehicle blueprints (Tesla or Audi)
    bp_lib = world.get_blueprint_library()
    vehicle_bp = bp_lib.find('vehicle.tesla.model3') if bp_lib.find('vehicle.tesla.model3') else bp_lib.filter('vehicle.*')[0]

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f"Spawned vehicle: {vehicle.type_id}")

    apply_weather_based_friction(vehicle, wetness, precipitation_deposits)

    move_spectator_to_vehicle(world, vehicle)

    vehicle.set_autopilot(False)
    control = carla.VehicleControl(throttle=0.7)
    vehicle.apply_control(control)
    print("Vehicle accelerating...")

    for _ in range(30):
        move_spectator_to_vehicle(world, vehicle)
        time.sleep(.25)

    vehicle.apply_control(carla.VehicleControl(throttle=1, steer=1.0))
    move_spectator_to_vehicle(world, vehicle)
    print("Vehicle turning...")
    for _ in range(10):
        #move_spectator_to_vehicle(world, vehicle)
        time.sleep(.25)

    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
    print("Braking and cleaning up...")
    time.sleep(2.0)
    vehicle.destroy()
    print("Done.")
