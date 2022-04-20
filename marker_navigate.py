import cv2
import cv2.aruco as aruco
import numpy as np
from djitellopy import Tello


marker_size = 20
camera_matrix = np.loadtxt('cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt('cameraDistortion.txt', delimiter=',')
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()


def move(tello, d, d2):
    # left right
    if d[0] > 30:
        lr = 50
    elif d[0] < -30:
        lr = -50
    elif d[0] > 5:
        lr = 20
    elif d[0] < -5:
        lr = -20
    else:
        lr = int(d[0])

    # up down
    if d[1] > 30:
        ud = -50
    elif d[1] < -30:
        ud = 50
    elif d[1] > 5:
        ud = -20
    elif d[1] < -5:
        ud = 20
    else:
        ud = int(-d[1])

    # forward backward
    if d[2] < 100 and -10 <= d[1] <= 10 and -10 <= d[0] <= 10:
        fb = -10
    elif -10 <= d[1] <= 10 and -10 <= d[0] <= 10:
        if d[2] > 200:
            fb = 50
            ud = ud - 25
        else:
            fb = 20
            ud = ud - 15
    else:
        fb = 0

    if d[2] < 150:
        if abs(d2[0]) > 5:
            yaw = int(d2[0] / 2)
        elif d2[0] > 0:
            yaw = 10
        elif d2[0] < 0:
            yaw = -10
        else:
            yaw = 0
    else:
        yaw = 0

    tello.send_rc_control(lr, fb, ud, yaw)


def droneControl():
    tello = Tello()
    tello.connect()
    tello.streamon()
    print("Battery : ", tello.get_battery(), "%")
    status = "back"
    stage = 0
    takeoff = 0

    while status == "back":
        frame = tello.get_frame_read().frame
        if takeoff == 0:
            tello.takeoff()
            takeoff = 1
        imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(image=imgGray, dictionary=aruco_dict, parameters=parameters,
                                                     cameraMatrix=camera_matrix, distCoeff=camera_distortion)

        if stage == 0:
            if ids is not None:
                stage = 1
            else:
                tello.send_rc_control(0, 0, 0, 25)

        print(ids)
        if stage == 1:
            if ids is not None:
                noneCount = 0
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
                str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
                cv2.putText(frame, str_position, (0, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2, cv2.LINE_AA)

                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T
                pos_camera = -R_tc * np.matrix(tvec).T

                str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (pos_camera[0], pos_camera[1], pos_camera[2])
                cv2.putText(frame, str_position, (0, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2, cv2.LINE_AA)

                move(tello, tvec, pos_camera)

                if abs(tvec[0]) <= 5 and abs(tvec[1]) <= 5 and tvec[2] <= 100 and abs(pos_camera[0]) <= 15:
                    tello.send_rc_control(0, 0, 0, 0)
                    stage = 2

            elif ids is None:
                tello.send_rc_control(0, 0, 0, 0)

        if stage == 2:
            tello.enable_mission_pads()
            tello.set_mission_pad_detection_direction(0)
            stage = 3

        if stage == 3:
            tello.go_xyz_speed_mid(0, 0, 100, 10, 1)
            tello.go_xyz_speed_mid(0, 0, 50, 10, 1)
            x = tello.get_mission_pad_distance_x()
            y = tello.get_mission_pad_distance_y()
            z = tello.get_mission_pad_distance_z()

            while abs(x - 20) >= 3 or abs(y) >= 3:
                tello.go_xyz_speed_mid(20, 0, 50, 10, 1)
                x = tello.get_mission_pad_distance_x()
                y = tello.get_mission_pad_distance_y()
                print(x, " ", y, " ", z)

            stage = 10

        cv2.imshow("Image", frame)

        if cv2.waitKey(1) == 27 or stage == 10:
            print("Battery : ", tello.get_battery(), "%")
            cv2.destroyAllWindows()
            tello.end()
            break
    print("landed")


if __name__ == '__main__':
    droneControl()
