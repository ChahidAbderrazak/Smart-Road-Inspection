import glob
import os
import sys
import time
import pygame
import random
import pickle
import pyrebase
import cv2

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np

dict_frame = {}  # dictionary of sensor data corresponding to a single frame
dict_fr_list = []  # list of individual dictionaries of sensors' data for all frames
data_root = 'D:\HAIS_DATA'  # storage path on local device
filenames_strg_list = []  # list of filenames stored on firebase corresponding to individual frames
configuration = {}  # input configuration
simulation_parameters = []

flag = 0

# firebase storage configuration

firebaseConfig = {'apiKey': "AIzaSyCBTp3caunMJ6JlXyNXlN0DERVLi8EJ6Ho",
                  'authDomain': "hais-project-d9692.firebaseapp.com",
                  'databaseURL': "https://hais-project-d9692-default-rtdb.firebaseio.com",
                  'projectId': "hais-project-d9692",
                  'storageBucket': "hais-project-d9692.appspot.com",
                  'messagingSenderId': "926868745531",
                  'appId': "1:926868745531:web:6073d481701edc27c51b1e",
                  'measurementId': "G-25QKT2MGB6"}

firebase = pyrebase.initialize_app(firebaseConfig)
storage = firebase.storage()


#########################  CARLA FUNCTIONS  ########################

# function to add data to main sensor data dictionary based on corresponding frame
def add_data_to_pipeline(frame):
    global dict_fr_list, dict_frame

    if dict_frame == {}:
        dict_frame['frame'] = frame

    elif frame != dict_frame['frame']:
        dict_fr_list.append(dict_frame)  # main list with all frames
        dict_frame = {}  # individual frame
        dict_frame['frame'] = frame

    return dict_frame


# function to generate storage path and filename for rgb camera images
def get_rgb_image_path(config, fr=''):
    if fr == '':
        filename_strg = 'image.png'
    else:
        filename_strg = fr + '.png'
    dir_storage = os.path.join(config['Scenario'], config['Used_Case'], 'RGB_Camera')
    return filename_strg, dir_storage


# function to generate storage path and filename for depth camera images
def get_depth_image_path(config, fr=''):
    if fr == '':
        filename_strg = 'image.png'
    else:
        filename_strg = fr + '.png'
    dir_storage = os.path.join(config['Scenario'], config['Used_Case'], 'Depth_Camera')
    return filename_strg, dir_storage


# function to convert .png file to numpy array
def image_to_array(path):
    try:
        img = cv2.imread(path, 0)

    except:
        msg = '\n Error: the image path ' + path + 'cannot be loaded!!!!'
        raise Exception(msg)

    return np.array(img)


# listen function for obstacle sensor
def obstacle_sensor_listen(obstacle_data):
    global dict_fr_list, dict_frame
    # dictionary of obstacle sensor data
    obstacle_sensor_dict = {'Timestamp': obstacle_data.timestamp, 'Actor': str(obstacle_data.actor),
                            'Other_Actor': str(obstacle_data.other_actor), 'Distance': obstacle_data.distance}

    # update outputs
    dict_frame = add_data_to_pipeline(obstacle_data.frame)
    dict_frame['Obstacle_Sensor'] = obstacle_sensor_dict


# listen function for front rgb camera
def rgb_camera_front_listen(image):
    global data_root, dict_fr_list, dict_frame, configuration

    # save image file locally 
    frame = str(image.frame)
    filename_strg, dir_storage = get_rgb_image_path(configuration, fr=frame)
    image_save_path = os.path.join(data_root, dir_storage, 'Front_Camera', filename_strg)
    image.save_to_disk(image_save_path)

    # save image in dict for firebase
    image_storage_path = os.path.join(data_root, image_save_path)
    rgb_image_array = image_to_array(image_storage_path)

    # dictionary of rgb camera
    rgb_camera_front_dict = {'Timestamp': image.timestamp, "Image": rgb_image_array}

    # update outputs
    dict_frame = add_data_to_pipeline(image.frame)
    dict_frame['RGB_Camera_Front'] = rgb_camera_front_dict


# listen function for back rgb camera
def rgb_camera_back_listen(image):
    global data_root, dict_fr_list, dict_frame, configuration

    # save image file locally 
    frame = str(image.frame)
    filename_strg, dir_storage = get_rgb_image_path(configuration, fr=frame)
    image_save_path = os.path.join(data_root, dir_storage, 'Back_Camera', filename_strg)
    image.save_to_disk(image_save_path)

    # save image in dict for firebase
    image_storage_path = os.path.join(data_root, image_save_path)
    rgb_image_array = image_to_array(image_storage_path)

    # dictionary of rgb camera
    rgb_camera_back_dict = {'Timestamp': image.timestamp, "Image": rgb_image_array}

    # update outputs
    dict_frame = add_data_to_pipeline(image.frame)
    dict_frame['RGB_Camera_Back'] = rgb_camera_back_dict


# listen function for depth camera
def depth_camera_listen(image):
    global data_root, dict_fr_list, dict_frame, configuration

    # save image file locally
    frame = str(image.frame)
    filename_strg, dir_storage = get_depth_image_path(configuration, fr=frame)
    image_save_path = os.path.join(data_root, dir_storage, filename_strg)
    image.save_to_disk(image_save_path)

    # save image in dict for firebase
    image_storage_path = os.path.join(data_root, image_save_path)
    depth_image_array = image_to_array(image_storage_path)

    # dictionary of depth camera
    depth_camera_dict = {'Timestamp': image.timestamp, "Image": depth_image_array}

    # update outputs
    dict_frame = add_data_to_pipeline(image.frame)
    dict_frame['Depth_Camera'] = depth_camera_dict


# listen function for imu sensor
def imu_sensor_listen(imu_sensor_data):
    global dict_fr_list, dict_frame

    # dictionary for imu sensor data
    imu_sensor_dict = {'Timestamp': imu_sensor_data.timestamp, 'Accelerometer': str(imu_sensor_data.accelerometer),
                       'Gyroscope': str(imu_sensor_data.gyroscope), 'Compass': str(imu_sensor_data.compass)}

    # update outputs
    dict_frame = add_data_to_pipeline(imu_sensor_data.frame)
    dict_frame['IMU_Sensor'] = imu_sensor_dict


# listen function for imu sensor
def gnss_sensor_listen(gnss_sensor_data):
    global dict_fr_list, dict_frame

    # dictionary for gnss sensor data
    gnss_sensor_dict = {'Timestamp': gnss_sensor_data.timestamp, 'Latitude': str(gnss_sensor_data.latitude),
                        'Longitude': str(gnss_sensor_data.longitude), 'Altitude': str(gnss_sensor_data.altitude)}

    # update outputs
    dict_frame = add_data_to_pipeline(gnss_sensor_data.frame)
    dict_frame['GNSS_Sensor'] = gnss_sensor_dict


# listen function for LIDAR
def lidar_sensor_listen(lidar_data):
    global dict_fr_list, dict_frame

    # dictionary for lidar sensor data
    lidar_sensor_dict = {'Timestamp': lidar_data.timestamp, 'Raw Data': np.array(lidar_data.raw_data),
                         'Horizontal Angle': lidar_data.horizontal_angle, 'Channels': lidar_data.channels,
                         'Number of points per channel captured this frame': lidar_data.get_point_count(
                             lidar_data.channels),
                         }

    # update outputs
    dict_frame = add_data_to_pipeline(lidar_data.frame)
    dict_frame['Lidar_Sensor'] = lidar_sensor_dict


# listen function for RADAR
def radar_sensor_listen(radar_data):
    global dict_fr_list, dict_frame

    # To get a numpy [[vel, azimuth, altitude, depth],...[,,,]]:
    points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (len(radar_data), 4))

    # dictionary for radar sensor data
    radar_sensor_dict = {'Timestamp': radar_data.timestamp, 'Raw Data': points}

    # update outputs
    dict_frame = add_data_to_pipeline(radar_data.frame)
    dict_frame['Radar_Sensor'] = radar_sensor_dict


# listen function for semantic LIDAR
def semantic_lidar_listen(semantic_lidar_data):
    global dict_fr_list, dict_frame

    # dictionary for semantic lidar sensor data
    semantic_lidar_sensor_dict = {'Timestamp': semantic_lidar_data.timestamp,
                                  'Raw Data': np.array(semantic_lidar_data.raw_data),
                                  'Horizontal Angle': semantic_lidar_data.horizontal_angle,
                                  'Channels': semantic_lidar_data.channels,
                                  'Number of points per channel captured this frame': semantic_lidar_data.get_point_count(
                                      semantic_lidar_data.channels)}

    # update outputs
    dict_frame = add_data_to_pipeline(semantic_lidar_data.frame)
    dict_frame['Semantic_Lidar_Sensor'] = semantic_lidar_sensor_dict


def get_parameters(config):
    if config['Scenario'] == 'S1':
        print('High Safety Index Scenario')
        cloudiness = random.randint(0, 30)
        precipitation = random.randint(0, 30)
        precipitation_deposits = random.randint(0, 30)
        wind_intensity = random.randint(0, 30)
        fog_density = random.randint(0, 30)
        wetness = random.randint(0, 30)
        sun_azimuth_angle = random.randint(0, 180)
        sun_altitude_angle = random.randint(-90, 90)
        n_car = 50
        tire_friction = 2.0

    elif config['Scenario'] == 'S2':
        print('Medium Safety Index Scenario')
        cloudiness = random.randrange(30, 60)
        precipitation = random.randrange(30, 60)
        precipitation_deposits = random.randrange(30, 60)
        wind_intensity = random.randrange(30, 60)
        fog_density = random.randrange(30, 60)
        wetness = random.randrange(30, 60)
        sun_azimuth_angle = random.randint(0, 180)
        sun_altitude_angle = random.randint(-90, 90)
        n_car = 100
        tire_friction = random.randrange(1.0, 2.0)

    elif config['Scenario'] == 'S3':
        print('Low Safety Index Scenario')
        cloudiness = random.randrange(60, 100)
        precipitation = random.randrange(60, 100)
        precipitation_deposits = random.randrange(60, 100)
        wind_intensity = random.randrange(60, 100)
        fog_density = random.randrange(60, 100)
        wetness = random.randrange(60, 100)
        sun_azimuth_angle = random.randint(0, 180)
        sun_altitude_angle = random.randint(-90, 90)
        n_car = 150
        tire_friction = random.uniform(0.5, 1.0)

    else:
        print("flag")
        raise ValueError('Error: scenario undefined!!!')

    return cloudiness, precipitation, precipitation_deposits, wind_intensity, fog_density, wetness, sun_azimuth_angle, sun_altitude_angle, n_car, tire_friction


def run_carla_experiment(config):
    global dict_fr_list, dict_frame, configuration, simulation_parameters
    actor_list = []
    cars_list = []
    configuration = config
    k = 1

    # CARLA set up
    client = carla.Client("localhost", 2000)
    client.set_timeout(1000.0)
    world = client.get_world()

    cloudiness, precipitation, precipitation_deposits, wind_intensity, fog_density, wetness, sun_azimuth_angle, sun_altitude_angle, n_car, tire_friction = get_parameters(
        config)

    simulation_parameters_dict = {'Time_frame': k, 'cloudiness': cloudiness, 'precipitation': precipitation,
                                  'precipitation_deposits': precipitation_deposits, 'wind_intensity': wind_intensity,
                                  'fog_density': fog_density, 'wetness': wetness,
                                  'sun_azimuth_angle': sun_azimuth_angle, 'sun_altitude_angle': sun_altitude_angle,
                                  'n_car': n_car, 'tire_friction': tire_friction}
    simulation_parameters.append(simulation_parameters_dict)
    weather1 = carla.WeatherParameters(cloudiness=cloudiness, precipitation=precipitation,
                                       precipitation_deposits=precipitation_deposits,
                                       wind_intensity=wind_intensity, fog_density=fog_density, wetness=wetness,
                                       sun_azimuth_angle=sun_azimuth_angle, sun_altitude_angle=sun_altitude_angle)
    world.set_weather(weather1)

    # spawn random cars
    bp_lib = world.get_blueprint_library().filter('vehicle.*.*')
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    for i in range(n_car):
        vehicle_bp = random.choice(bp_lib)
        vehicle_i = world.spawn_actor(vehicle_bp, spawn_points[i])
        vehicle_i.set_autopilot(True)
        actor_list.append(vehicle_i)
        cars_list.append(vehicle_i)

    # spawn HAIS car
    hais_bp = world.get_blueprint_library().filter("model3")[0]
    hais_transform = carla.Transform(carla.Location(x=-75.4 - 10.0, y=-37.0, z=15),
                                     carla.Rotation(pitch=0, yaw=90, roll=0))
    hais_vehicle = world.spawn_actor(hais_bp, hais_transform)
    hais_vehicle.set_autopilot(True)
    actor_list.append(hais_vehicle)
    hais_vehicle.apply_physics_control(carla.VehiclePhysicsControl(
        wheel=[carla.WheelPhysicsControl(tire_friction=tire_friction),
               carla.WheelPhysicsControl(tire_friction=tire_friction),
               carla.WheelPhysicsControl(tire_friction=tire_friction),
               carla.WheelPhysicsControl(tire_friction=tire_friction)]))
    hais_vehicle.set_autopilot(True)
    # spawn and attach sensors
    n_sensor = 1  # number of sensors on each CAV
    for i in range(n_sensor):
        bp = world.get_blueprint_library().find('sensor.other.obstacle')
        bp.set_attribute('distance', '100.0')
        bp.set_attribute('hit_radius', '1.0')
        bp.set_attribute('sensor_tick', '5.0')
        transform = carla.Transform(carla.Location(x=3.0, y=0.0, z=1.0), carla.Rotation(pitch=0, yaw=0, roll=0))
        obstacle_sensor = world.spawn_actor(bp, transform, attach_to=hais_vehicle)
        actor_list.append(obstacle_sensor)

        bp1 = world.get_blueprint_library().find('sensor.camera.rgb')
        bp1.set_attribute('sensor_tick', '5.0')
        transform1 = carla.Transform(carla.Location(x=3.0, y=0.0, z=1.0), carla.Rotation(pitch=0, yaw=0, roll=0))
        rgb_camera_front = world.spawn_actor(bp1, transform1, attach_to=hais_vehicle)
        actor_list.append(rgb_camera_front)

        bp8 = world.get_blueprint_library().find('sensor.camera.rgb')
        bp8.set_attribute('sensor_tick', '5.0')
        transform8 = carla.Transform(carla.Location(x=-3.0, y=0.0, z=1.0), carla.Rotation(pitch=0, yaw=180, roll=0))
        rgb_camera_back = world.spawn_actor(bp8, transform8, attach_to=hais_vehicle)
        actor_list.append(rgb_camera_back)

        bp2 = world.get_blueprint_library().find('sensor.camera.depth')
        bp2.set_attribute('sensor_tick', '5.0')
        transform2 = carla.Transform(carla.Location(x=3.0, y=0.0, z=1.0), carla.Rotation(pitch=0, yaw=0, roll=0))
        depth_camera = world.spawn_actor(bp2, transform2, attach_to=hais_vehicle)
        actor_list.append(depth_camera)

        bp3 = world.get_blueprint_library().find('sensor.other.imu')
        bp3.set_attribute('sensor_tick', '5.0')
        transform3 = carla.Transform(carla.Location(), carla.Rotation(pitch=0, yaw=0, roll=0))
        imu_sensor = world.spawn_actor(bp3, transform3, attach_to=hais_vehicle)
        actor_list.append(imu_sensor)

        bp4 = world.get_blueprint_library().find('sensor.other.gnss')
        bp4.set_attribute('sensor_tick', '5.0')
        transform4 = carla.Transform(carla.Location(), carla.Rotation(pitch=0, yaw=0, roll=0))
        gnss_sensor = world.spawn_actor(bp4, transform4, attach_to=hais_vehicle)
        actor_list.append(gnss_sensor)

        bp5 = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp5.set_attribute('sensor_tick', '5.0')
        transform5 = carla.Transform(carla.Location(x=3.0, y=0.0, z=1.0), carla.Rotation(pitch=-90, yaw=0, roll=0))
        lidar_sensor = world.spawn_actor(bp5, transform5, attach_to=hais_vehicle)
        actor_list.append(lidar_sensor)

        bp6 = world.get_blueprint_library().find('sensor.other.radar')
        bp6.set_attribute('sensor_tick', '5.0')
        transform6 = carla.Transform(carla.Location(x=3.0, y=0.0, z=1.0), carla.Rotation(pitch=-90, yaw=0, roll=0))
        radar_sensor = world.spawn_actor(bp6, transform6, attach_to=hais_vehicle)
        actor_list.append(radar_sensor)

        bp7 = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        bp7.set_attribute('sensor_tick', '5.0')
        transform7 = carla.Transform(carla.Location(x=3.0, y=0.0, z=1.0), carla.Rotation(pitch=-90, yaw=0, roll=0))
        semantic_lidar_sensor = world.spawn_actor(bp7, transform7, attach_to=hais_vehicle)
        actor_list.append(semantic_lidar_sensor)

    obstacle_sensor.listen(obstacle_sensor_listen)
    rgb_camera_front.listen(rgb_camera_front_listen)
    rgb_camera_back.listen(rgb_camera_back_listen)
    depth_camera.listen(depth_camera_listen)
    imu_sensor.listen(imu_sensor_listen)
    gnss_sensor.listen(gnss_sensor_listen)
    lidar_sensor.listen(lidar_sensor_listen)
    radar_sensor.listen(radar_sensor_listen)
    semantic_lidar_sensor.listen(semantic_lidar_listen)

    pygame.init()
    clock = pygame.time.Clock()
    current_time = pygame.time.get_ticks()
    while pygame.time.get_ticks() < config['duration'] * 1000:
        for car in cars_list:
            car.set_autopilot(True)
        hais_vehicle.set_autopilot(True)
        if pygame.time.get_ticks() - current_time > 20.0 * 1000:
            cloudiness, precipitation, precipitation_deposits, wind_intensity, fog_density, wetness, sun_azimuth_angle, sun_altitude_angle, n_car, tire_friction = get_parameters(
                config)
            k=k+1
            print(k)
            simulation_parameters_dict = {'Time_frame': k, 'cloudiness': cloudiness, 'precipitation': precipitation,
                                          'precipitation_deposits': precipitation_deposits,
                                          'wind_intensity': wind_intensity,
                                          'fog_density': fog_density, 'wetness': wetness,
                                          'sun_azimuth_angle': sun_azimuth_angle,
                                          'sun_altitude_angle': sun_altitude_angle,
                                          'n_car': n_car, 'tire_friction': tire_friction}
            simulation_parameters.append(simulation_parameters_dict)
            weather1 = carla.WeatherParameters(cloudiness=cloudiness, precipitation=precipitation,
                                               precipitation_deposits=precipitation_deposits,
                                               wind_intensity=wind_intensity, fog_density=fog_density,
                                               wetness=wetness)
            world.set_weather(weather1)
            current_time = pygame.time.get_ticks()

    obstacle_sensor.stop()
    rgb_camera_front.stop()
    rgb_camera_back.stop()
    depth_camera.stop()
    imu_sensor.stop()
    gnss_sensor.stop()
    lidar_sensor.stop()
    radar_sensor.stop()

    for car in cars_list:
        car.set_autopilot(False)
    hais_vehicle.set_autopilot(False)

    # destroy actors spawned in CARLA
    for actor in actor_list:
        actor.destroy()
    print(" All cleaned up!")
    pygame.quit()
    return simulation_parameters, dict_fr_list


#########################  FIREBASE FUNCTIONS  ########################


# convert pkl file to python dictionary
def pkl_to_dict(filename):
    open_file = open(filename, "rb")
    dict = pickle.load(open_file)
    open_file.close()
    return dict


# convert python dictionary to pkl file
def dict_to_pkl(filename, dict):
    """
     Save stored  variables list <var_list> in <filename>:
     save_variables(filename, var_list)
    """

    open_file = open(filename, "wb");
    pickle.dump(dict, open_file);
    open_file.close()


# get file name
def get_file_names(config, fr=''):
    if fr == '':
        filename_strg = config['Scenario'] + '_' + config['Used_Case'] + '_.pkl'
    else:
        filename_strg = config['Scenario'] + '_' + config['Used_Case'] + '_Fr' + fr + '_.pkl'
    dir_storage = os.path.join(config['Scenario'], config['Used_Case'])
    return filename_strg, dir_storage


# Upload data on firebase
def push_data_to_firebase(config, dict_fr_list_push, simulation_parameters):
    global filenames_strg_list, flag, storage
    # print(f'\n config={config} \n dict_fr_list={dict_fr_list} ')

    # flag: if error occurs  in the next line, then break it into for loop 
    try:
        filename_strg, dir_storage = get_file_names(config)
        dict_to_pkl(filename_strg, dict_fr_list_push)
        dict_to_pkl('simulation_parameters.pkl', simulation_parameters)
        storage.child(config['Scenario']).child(config['Used_Case']).child(filename_strg).put(filename_strg)
        storage.child(config['Scenario']).child(config['Used_Case']).child('simulation_parameters.pkl').put('simulation_parameters.pkl')
        flag = 1

    except:
        for dict in dict_fr_list_push:
            filename_strg, dir_storage = get_file_names(config, fr=str(dict['frame']))
            dict_to_pkl(filename_strg, dict)
            dict_to_pkl('simulation_parameters.pkl', simulation_parameters)
            filenames_strg_list.append(filename_strg)
            storage.child(config['Scenario']).child(config['Used_Case']).child(filename_strg).put(filename_strg)
            storage.child(config['Scenario']).child(config['Used_Case']).child('simulation_parameters.pkl').put(
                'simulation_parameters.pkl')
            flag = 0


# Retrieve data from firebase
def retreive_data_from_firebase(config):
    global filenames_strg_list, flag, storage
    dict_fr_list_retrieved = []
    if flag == 1:
        filename_strg, dir_storage = get_file_names(config)
        storage.child(config['Scenario']).child(config['Used_Case']).child(filename_strg).download(filename_strg)
        # storage.child(os.path.join(dir_storage, filename_strg)).download(filename_strg)
        dict_fr_list_retrieved = pkl_to_dict(filename_strg)

    elif flag == 0:
        filename_strg, dir_storage = get_file_names(config)
        for filename in filenames_strg_list:
            storage.child(config['Scenario']).child(config['Used_Case']).child(filename_strg).download(filename_strg)
            # storage.child(os.path.join(dir_storage, filename)).download(filename)
            dict_frame_retrieved = pkl_to_dict(filename)
            dict_fr_list_retrieved.append(dict_frame_retrieved)

    return dict_fr_list_retrieved
