from djitellopy import Tello
from datetime import datetime
import cv2, threading, concurrent.futures, keyboard, time

# Read from file
# once done... write done: write to file
# if battery dies: if line contains "done": skip line
    # if does not contain "done": continue ..... calibration/smart marks/sensor reading accuracy

'''
Environment Measurements
length: rightside=155cm
width: 52cm
height: 108cm
home-depot-orange-sign: 24.5cm x 24.5cm
tube-diameter: 15cm

'''
drone = Tello()
drone.connect()

keepRecording = True
drone.streamon()
frame_read = drone.get_frame_read()

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))

    while keepRecording:
        video.write(frame_read.frame)
        time.sleep(1 / 30)

    video.release()
    print('video released')

def live_stream():
    while keepRecording:
        img = frame_read.frame
        img = cv2.resize(img, (700, 700))
        cv2.imshow("Live Stream", img)
        cv2.waitKey(1)
    print('live stream ended')

def drone_state():
    print(drone.get_current_state())

def sequence():
    print('*'*10, 'Flight Path Starting...', '*'*10)
    drone.takeoff()
    drone.move_up(30)
    time.sleep(1)
    
    print('*'*10, 'Round 1', '*'*10)
    drone.move_right(175)
    drone.rotate_counter_clockwise(32)
    drone.move_right(130)
    drone.rotate_counter_clockwise(62)
    drone.move_right(180)
    drone.rotate_counter_clockwise(62)
    drone.move_right(130)
    drone.rotate_counter_clockwise(32)
    drone.move_right(175)
    
    drone.move_down(20)
    time.sleep(1)

    print('*'*10, 'Round 2', '*'*10)
    drone.move_left(150)
    drone.rotate_clockwise(32)
    drone.move_left(100)
    drone.rotate_clockwise(62)
    drone.move_left(150)
    drone.rotate_clockwise(62)
    drone.move_left(100)
    drone.rotate_clockwise(32)
    drone.move_left(175)

    drone.move_down(20)
    time.sleep(1)

    print('*'*10, 'Round 3', '*'*10)
    drone.move_right(175)
    drone.rotate_counter_clockwise(32)
    drone.move_right(130)
    drone.rotate_counter_clockwise(62)
    drone.move_right(180)
    drone.rotate_counter_clockwise(62)
    drone.move_right(130)
    drone.rotate_counter_clockwise(32)
    drone.move_right(175)

    drone.move_down(20)
    time.sleep(1)

    print('*'*10, 'Round 4', '*'*10)
    drone.move_left(150)
    drone.rotate_clockwise(32)
    drone.move_left(100)
    drone.rotate_clockwise(62)
    drone.move_left(150)
    drone.rotate_clockwise(62)
    drone.move_left(100)
    drone.rotate_clockwise(32)
    drone.move_left(175)

    drone.move_down(20)
    time.sleep(1)

    print('*'*10, 'Round 5', '*'*10)
    drone.move_right(175)
    drone.rotate_counter_clockwise(32)
    drone.move_right(130)
    drone.rotate_counter_clockwise(62)
    drone.move_right(180)
    drone.rotate_counter_clockwise(62)
    drone.move_right(130)
    drone.rotate_counter_clockwise(32)
    drone.move_right(175)

    drone.move_down(20)
    time.sleep(1)

    print('*'*10, 'Round 6', '*'*10)
    drone.move_left(150)
    drone.rotate_clockwise(32)
    drone.move_left(100)
    drone.rotate_clockwise(62)
    drone.move_left(150)
    drone.rotate_clockwise(62)
    drone.move_left(100)
    drone.rotate_clockwise(32)
    drone.move_left(175)

    time.sleep(1)
    drone.land()
    

def battery():
    while True:
        # print('battery:', drone.get_battery())
        drone.send_read_command('battery?')
        time.sleep(1)



recorder = threading.Thread(target=videoRecorder)
# live_stream_thread = threading.Thread(target=live_stream)

print('=*'*10, drone.get_battery(), '% =*'*10)

# time.sleep(5)
# live_stream_thread.start()
recorder.start()

# time.sleep(5)

sequence()

keepRecording = False
# live_stream_thread.join()
recorder.join()
print('=*'*10, drone.get_battery(), '% =*'*10)
print('program ended')
exit(0)

# start_time = time.perf_counter()
# finish_time = time.perf_counter()
# print(f'Mission Execution Time: {round(finish_time-start_time, 2)} seconds')