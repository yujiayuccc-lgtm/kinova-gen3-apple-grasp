#!/usr/bin/env python3
import json
import math
import os
import re
import subprocess

import cv2
import numpy as np


ROBOT_IP = '192.168.1.10'
COLOR_FILE = '/tmp/apple_locator_color.jpg'
DEPTH_FILE = '/tmp/apple_locator_depth.raw'
ANNOTATED_FILE = '/tmp/apple_locator_annotated.jpg'


def run(command, timeout=20):
    env = os.environ.copy()
    for key in ('ALL_PROXY', 'all_proxy', 'HTTP_PROXY', 'http_proxy',
                'HTTPS_PROXY', 'https_proxy', 'FTP_PROXY', 'ftp_proxy'):
        env.pop(key, None)
    env['NO_PROXY'] = f'{ROBOT_IP},localhost,127.0.0.1'
    return subprocess.run(command, env=env, timeout=timeout, check=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def capture_rgbd():
    run(['gst-launch-1.0', '-q', 'rtspsrc', f'location=rtsp://{ROBOT_IP}/color',
         'protocols=tcp', 'latency=100', '!', 'rtph264depay', '!', 'avdec_h264',
         '!', 'videoconvert', '!', 'jpegenc', 'snapshot=true', '!', 'filesink',
         f'location={COLOR_FILE}'])
    run(['gst-launch-1.0', '-q', 'rtspsrc', f'location=rtsp://{ROBOT_IP}/depth',
         'protocols=tcp', 'latency=100', '!', 'rtpgstdepay', '!',
         'video/x-raw,format=GRAY16_LE,width=480,height=270', '!', 'identity',
         'eos-after=2', '!', 'filesink', f'location={DEPTH_FILE}'])


def quaternion_matrix(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    s = 2.0 / n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]])


def current_camera_transform():
    proc = subprocess.run(
        ['timeout', '4', 'ros2', 'run', 'tf2_ros', 'tf2_echo',
         'base_link', 'end_effector_link'], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, check=False)
    translations = re.findall(r'Translation: \[([^]]+)\]', proc.stdout)
    quaternions = re.findall(r'Quaternion \(xyzw\) \[([^]]+)\]', proc.stdout)
    if not translations or not quaternions:
        raise RuntimeError('cannot read base_link -> end_effector_link transform')
    te = np.fromstring(translations[-1], sep=',')
    q = np.fromstring(quaternions[-1], sep=',')
    re_mat = quaternion_matrix(*q)
    # Kinova real-hardware URDF mount: xyz=(0,.05639,-.00305), rpy=(pi,pi,0).
    rm = np.diag([-1.0, -1.0, 1.0])
    tm = np.array([0.0, 0.05639, -0.00305])
    return re_mat @ rm, te + re_mat @ tm


def locate():
    color = cv2.imread(COLOR_FILE)
    depth = np.fromfile(DEPTH_FILE, dtype='<u2').reshape(270, 480)
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 80, 40), (15, 255, 255))
    mask |= cv2.inRange(hsv, (165, 80, 40), (179, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > 3000]
    if not contours:
        raise RuntimeError('red apple was not detected')
    contour = max(contours, key=cv2.contourArea)
    bx, by, bw, bh = cv2.boundingRect(contour)
    region = np.zeros_like(mask)
    cv2.drawContours(region, [contour], -1, 255, -1)
    moments = cv2.moments(contour)
    pixel = [moments['m10']/moments['m00'], moments['m01']/moments['m00']]

    vd, ud = np.indices(depth.shape)
    z = depth.astype(float) / 1000.0
    good = (z > 0.12) & (z < 0.45)
    safe_z = np.where(z > 0, z, 1.0)
    xd = (ud - 243.87228) * safe_z / 360.01333
    yd = (vd - 137.9218444) * safe_z / 360.013366699
    # Official depth-frame transform relative to color/camera_link.
    xc, yc, zc = xd - 0.0195, yd - 0.005, safe_z
    uc = np.rint(1297.672904 * xc/zc + 620.914026).astype(int)
    vc = np.rint(1298.631344 * yc/zc + 238.280325).astype(int)
    good &= (uc >= 0) & (uc < 1280) & (vc >= 0) & (vc < 720)
    selected = good.copy()
    selected[good] = region[vc[good], uc[good]] > 0
    pc = np.column_stack((xc[selected], yc[selected], z[selected]))
    if len(pc) < 1000:
        raise RuntimeError('insufficient apple depth points')

    rb, tb = current_camera_transform()
    points = pc @ rb.T + tb
    points = points[(points[:, 2] > 0.01) & (points[:, 2] < 0.10)]
    keep = np.ones(len(points), dtype=bool)
    for _ in range(6):
        q = points[keep]
        a = np.column_stack((2*q, np.ones(len(q))))
        b = np.sum(q*q, axis=1)
        solution = np.linalg.lstsq(a, b, rcond=None)[0]
        center = solution[:3]
        radius = math.sqrt(max(0.0, solution[3] + center @ center))
        residual = np.abs(np.linalg.norm(points-center, axis=1) - radius)
        median = np.median(residual)
        keep = residual < max(0.003, 2.5*median)

    # Table height is the trusted z datum. Sphere z is more sensitive to the
    # camera mounting tolerance, so use the measured radius above z=0.
    center[2] = radius
    # Learned tool offset from the two successful physical grasps.
    wrist = center + np.array([-0.045, 0.015, 0.110])
    annotated = color.copy()
    cv2.drawContours(annotated, [contour], -1, (0, 255, 0), 3)
    cv2.circle(annotated, tuple(np.rint(pixel).astype(int)), 8, (255, 0, 0), -1)
    cv2.imwrite(ANNOTATED_FILE, annotated)
    return {
        'apple_base_m': {'x': float(center[0]), 'y': float(center[1]), 'z': float(center[2])},
        'apple_radius_m': float(radius),
        'pixel_center': {'u': float(pixel[0]), 'v': float(pixel[1])},
        'contour_bbox': {'x': int(bx), 'y': int(by), 'width': int(bw), 'height': int(bh)},
        'grasp_wrist_m': {'x': float(wrist[0]), 'y': float(wrist[1]), 'z': 0.150},
        'fit_points': int(keep.sum()),
        'fit_median_residual_m': float(np.median(residual[keep])),
        'annotated_image': ANNOTATED_FILE,
    }


if __name__ == '__main__':
    capture_rgbd()
    print(json.dumps(locate(), indent=2, ensure_ascii=False))
