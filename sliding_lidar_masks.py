from typing import Iterable, Optional
from collections import deque

from masks import ProbLidarMask, BoolLidarMask

def sliding_prob_lidar_mask(
            masks: Iterable[BoolLidarMask],
            window_size: int,
        ) -> Iterable[ProbLidarMask]:
    window = deque(maxlen=None if window_size is None else window_size+1)
    prob_mask : ProbLidarMask = ProbLidarMask.total_mask(0.0)


    for mask in masks:
        new_mask = ProbLidarMask(mask.map_poly(
            lambda x: 1.0 if x else 0.0
        ))
        window.append(new_mask)
        
        if len(window) > window_size:
            # print("dropping left")
            # Incremental version -- annoyingly broken
            dropped_mask = window.popleft()            

            prob_mask += (1.0/window_size)*(new_mask - dropped_mask) # type: ignore

            # prob_mask = (1.0/len(window))*sum(
            #     window,
            #     ProbLidarMask.total_mask(0.0),
            # )
        else:
            # print("straight computations")
            # Compute the mask manually during the warmup
            prob_mask = (1.0/len(window))*sum(
                window,
                ProbLidarMask.total_mask(0.0),
            )

        yield prob_mask

def sliding_lidar_mask(
            masks: Iterable[BoolLidarMask],
            window_size : int,
            cutoff : float,
        ) -> Iterable[BoolLidarMask]:
    for m in sliding_prob_lidar_mask(masks, window_size=window_size):
        yield m >= cutoff