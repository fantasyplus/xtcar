#!/usr/bin/env python
"""
   Tracking of rotating point.
   Rotation speed is constant.
   Both state and measurements vectors are 1D (a point angle),
   Measurement is the real point angle + gaussian noise.
   The real and the estimated points are connected with yellow line segment,
   the real and the measured points are connected with red line segment.
   (if Kalman filter works correctly,
    the yellow segment should be shorter than the red one,
    因为kalman滤波的最终更新公式为一个加权和。
    而参与加权和的有两个值：状态更新值和测量值。
    这里因为没有传感器获取观测量，就自己生成了一个传感器的观测量,
    生成这个观测量的时候，我们总是让这个观测量大于状态更新值了,
    所以，在状态更新值与观测量之间加权和，得到的数肯定小于观测量。
    所以，黄线肯定比红线短).
   Pressing any key (except ESC) will reset the tracking with a different speed.
   Pressing ESC will stop the program.
"""
# Python 2/3 compatibility
import sys
PY3 = sys.version_info[0] == 3

if PY3:
    long = int

import numpy as np
import cv2 as cv

from math import cos, sin, sqrt
import numpy as np

def main():
    img_height = 500
    img_width = 500
    # 这个2,1,0分别是啥意思？对应公式中哪些变量？
    kalman = cv.KalmanFilter(2, 1, 0)

    code = long(-1)

    cv.namedWindow("Kalman")

    while True:
        # 状态是两个值，速度和角度？
        state = 0.1 * np.random.randn(2, 1)

        # F 矩阵
        kalman.transitionMatrix = np.array([[1., 1.], [0., 1.]])
        # H 矩阵
        kalman.measurementMatrix = 1. * np.ones((1, 2))
        # 均值为零，符合高斯分布的白噪声
        kalman.processNoiseCov = 1e-5 * np.eye(2)
        # 均值为零，符合高斯分布的白噪声
        kalman.measurementNoiseCov = 1e-1 * np.ones((1, 1))

        kalman.errorCovPost = 1. * np.ones((2, 2))
        kalman.statePost = 0.1 * np.random.randn(2, 1)

        while True:
            def calc_point(angle):
                # 由一个角度值，计算出在平面上的具体的坐标值。
                return (np.around(img_width/2 + img_width/3*cos(angle), 0).astype(int),
                        np.around(img_height/2 - img_width/3*sin(angle), 1).astype(int))

            # 取得state的第一个值，作为角度
            state_angle = state[0, 0]
            state_pt = calc_point(state_angle)

            # 利用kalman滤波器的更新公式18,预测出下一时刻的角度。
            prediction = kalman.predict()
            predict_angle = prediction[0, 0]
            # 将角度换算为点
            predict_pt = calc_point(predict_angle)

            measurement = kalman.measurementNoiseCov * np.random.randn(1, 1)

            # generate measurement
            # 这个measurement最终是测量值zk
            # 这里没有传感器，就估计了传感器给出的角度
            measurement = np.dot(kalman.measurementMatrix, state) + measurement

            measurement_angle = measurement[0, 0]
            measurement_pt = calc_point(measurement_angle)

            # plot points
            # 绘制加号
            def draw_cross(center, color, d):
                cv.line(img,
                         (center[0] - d, center[1] - d), (center[0] + d, center[1] + d),
                         color, 1, cv.LINE_AA, 0)
                cv.line(img,
                         (center[0] + d, center[1] - d), (center[0] - d, center[1] + d),
                         color, 1, cv.LINE_AA, 0)

            img = np.zeros((img_height, img_width, 3), np.uint8)
            #k-1时刻的位置,白色
            draw_cross(np.int32(state_pt), (255, 255, 255), 3)
            #k-1时刻预测的测量值,预测的位置，就是由公式8计算到的值,红色
            draw_cross(np.int32(measurement_pt), (0, 0, 255), 3)
            #kalman滤波器预测的k时刻的位置,绿色
            draw_cross(np.int32(predict_pt), (0, 255, 0), 3)

            cv.line(img, state_pt, measurement_pt, (0, 0, 255), 3, cv.LINE_AA, 0)
            # 黄色
            cv.line(img, state_pt, predict_pt, (0, 255, 255), 3, cv.LINE_AA, 0)

            kalman.correct(measurement)

            # 点在下一时刻，实际所在的位置，这里还是用噪声模型了机器人实际的位置
            process_noise = sqrt(kalman.processNoiseCov[0,0]) * np.random.randn(2, 1)
            state = np.dot(kalman.transitionMatrix, state) + process_noise

            cv.imshow("Kalman", img)

            code = cv.waitKey(100)
            if code != -1:
                # 按任意值，跳出当前循环，重新设置state的值，即，重新设置点的位置和速度
                break

        if code in [27, ord('q'), ord('Q')]:
            break

    print('Done')


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
