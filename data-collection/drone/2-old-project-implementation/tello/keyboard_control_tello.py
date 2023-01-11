import keyboard_module as km
from djitellopy import Tello
from threading import Thread
import cv2, time

# facebook live, weibo, qqzone, customrtmp

flag = True
km.init()

drone = Tello()
drone.connect()
print(drone.get_battery())

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

def get_input():
    left_right, forward_backward, up_down, yaw_val = 0, 0, 0, 0
    velocity = 30 # HALF SPEED
    global flag

    if km.get_key('LEFT'): left_right = -velocity # LEFT
    elif km.get_key('RIGHT'): left_right = velocity # RIGHT

    if km.get_key('UP'): forward_backward = velocity # FORWARD
    elif km.get_key('DOWN'): forward_backward = -velocity # BACKWARD

    if km.get_key('a'): yaw_val = -velocity # YAW LEFT
    elif km.get_key('d'): yaw_val = velocity # YAW RIGHT

    if km.get_key('w'): up_down = velocity # UP
    elif km.get_key('s'): up_down = -velocity # DOWN

    if km.get_key('q'): drone.takeoff()
    if km.get_key('e'): drone.land()

    if km.get_key('o'): flag = False

    return [left_right, forward_backward, up_down, yaw_val]


recorder = Thread(target=videoRecorder)
recorder.start()

while flag == True:
    key_input = get_input()
    drone.send_rc_control(key_input[0], key_input[1], key_input[2], key_input[3])

    img = frame_read.frame
    img = cv2.resize(img, (800, 800))
    cv2.imshow("Live Stream", img)
    cv2.waitKey(1)

keepRecording = False
recorder.join()
print('program ended')
exit(0)