import math

import pytest
import itertools

from masks import (BoolLidarMask, ProbLidarMask)
from sliding_lidar_masks import (sliding_lidar_mask, sliding_prob_lidar_mask)

@pytest.fixture
def input_masks():
    return [
        BoolLidarMask([(2*k*math.pi/12, (2*k+5)*math.pi/12)])
            for k in range(12)
    ]

def to_prob_masks(ms):
    return [
        ProbLidarMask(m.map_poly(lambda x: 1.0 if x else 0.0))
            for m in ms
    ]


def sliding_window_manual(xs, window_size):
    for i in range(len(xs), 1):
        window = xs[min(i-window_size, 0): i+1]
        yield (1/len(window))*sum(window, ProbLidarMask.total_mask(0.0))


class TestSlidingLidarMasks:
    def test_sliding_window(self, input_masks):
        for x, y in zip(sliding_prob_lidar_mask(input_masks, 3),
                        sliding_window_manual(to_prob_masks(input_masks), 3)):
            assert x == y

    def test_sliding_mask(self, input_masks):
        for x, y in zip(sliding_lidar_mask(input_masks, 3, 0.5),
                        sliding_window_manual(to_prob_masks(input_masks), 3)):
            assert x == (y > 0.5)