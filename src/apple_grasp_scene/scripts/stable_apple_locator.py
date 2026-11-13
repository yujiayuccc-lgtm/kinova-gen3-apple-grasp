#!/usr/bin/env python3
import json
import sys

import numpy as np

import apple_locator


FRAME_COUNT = 3
MAX_XY_SPREAD_M = 0.008
MAX_RADIUS_SPREAD_M = 0.012
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
EDGE_MARGIN_PX = 5


def main():
    samples = []
    for index in range(FRAME_COUNT):
        apple_locator.capture_rgbd()
        sample = apple_locator.locate()
        bbox = sample['contour_bbox']
        if (bbox['x'] <= EDGE_MARGIN_PX or bbox['y'] <= EDGE_MARGIN_PX or
                bbox['x'] + bbox['width'] >= IMAGE_WIDTH - EDGE_MARGIN_PX or
                bbox['y'] + bbox['height'] >= IMAGE_HEIGHT - EDGE_MARGIN_PX):
            raise RuntimeError(f'apple contour touches image edge in frame {index + 1}')
        samples.append(sample)

    xyzr = np.array([[
        s['apple_base_m']['x'], s['apple_base_m']['y'],
        s['apple_base_m']['z'], s['apple_radius_m']
    ] for s in samples])
    xy_spread = np.ptp(xyzr[:, :2], axis=0)
    radius_spread = float(np.ptp(xyzr[:, 3]))
    if np.any(xy_spread > MAX_XY_SPREAD_M):
        raise RuntimeError(f'apple position is unstable across frames: xy spread={xy_spread.tolist()}')
    if radius_spread > MAX_RADIUS_SPREAD_M:
        raise RuntimeError(f'apple radius is unstable across frames: spread={radius_spread}')

    median = np.median(xyzr, axis=0)
    x, y, _, radius = [float(v) for v in median]
    result = {
        'apple_base_m': {'x': x, 'y': y, 'z': radius},
        'apple_radius_m': radius,
        'pixel_center': {
            'u': float(np.median([s['pixel_center']['u'] for s in samples])),
            'v': float(np.median([s['pixel_center']['v'] for s in samples])),
        },
        'grasp_wrist_m': {'x': x - 0.045, 'y': y + 0.015, 'z': 0.150},
        'fit_points': int(min(s['fit_points'] for s in samples)),
        'fit_median_residual_m': float(max(s['fit_median_residual_m'] for s in samples)),
        'frame_count': FRAME_COUNT,
        'xy_spread_m': {'x': float(xy_spread[0]), 'y': float(xy_spread[1])},
        'radius_spread_m': radius_spread,
        'radius_samples_m': [float(v) for v in xyzr[:, 3]],
        'annotated_image': apple_locator.ANNOTATED_FILE,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'STABLE_VISION_REJECTED: {exc}', file=sys.stderr)
        raise SystemExit(3)
