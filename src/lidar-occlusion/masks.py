# LIDAR masking

# import typing
from typing import Union, TypeVar, Tuple, Generic, List, Callable, Iterable, Type, Optional
from decimal import Decimal
from fractions import Fraction

NumberSpec = float | Decimal | str | Fraction

D = TypeVar('D')
K = TypeVar('K')
E = TypeVar('E')

from abc import ABCMeta
import math
import operator
from functools import partial
from collections.abc import Sequence
from collections import deque

import numpy as np
import copy

import portion

class LidarMask(Generic[D]):
    '''Signal of type [0, 2pi] -> D representing a Lidar occlusion mask.'''
    # Keys should be type T
    _values: np.ndarray
    # Value in pi radians
    _base_angle: Fraction
    
    def __init__(self, contents_spec,
                 base_angle : Optional[NumberSpec] = None):
        match base_angle:
            case None:
                self._base_angle = Fraction(1, 180)
            case _:
                self._base_angle = Fraction(base_angle)

        match contents_spec:
            case LidarMask():
                # Special case handling of coersing from another LidarMask
                assert base_angle is None
                self._base_angle = contents_spec._base_angle
                self._values = copy.copy(contents_spec._values)
            case np.ndarray():
                self._values = contents_spec
            case _:
                contents = list(contents_spec)

                if len(contents) == 0:
                    self._values = np.array([
                        self.default_value
                            for _ in range(round(2/self._base_angle))
                    ])
                else:
                    # Assume that the input is homogeneous
                    match contents[0]:
                        # Interval dict spec of form
                        # [(Interval_1, k_1), ..., (Interval_n, k_n)]
                        case ((_, _), _):
                            def min_or_default(xs : Iterable[D]) -> Optional[D]:
                                try:
                                    return min(xs) # type: ignore
                                except ValueError:
                                    return self.default_value

                            self._values = np.array([
                                min_or_default(k for ((l, u), k) in contents
                                    if l <= i*self._base_angle <= u)
                                for i in range(round(2/self._base_angle))
                            ])
                        # Boolean interval spec of form
                        # [Interval_1, ..., Interval_n]
                        case (_, _):
                            self._values = np.array([
                                len([None for (l, u) in contents
                                        if l <= i*self._base_angle <= u]) > 0
                                for i in range(round(2/self._base_angle))
                            ])
                        # A preformatted numpy array
                        case _:
                            self._values = np.array(contents_spec)
        
        assert self._values.shape == (round(2/self._base_angle),)

    @property
    def angles(self) -> List[Fraction]:
        return [i*self.base_angle for i in range(round(2*math.pi/self.base_angle))]

    @property
    def num_points(self):
        return len(self._values)

    @classmethod
    def total_mask(cls : Type[D],
                   value : D,
                   base_angle: Optional[NumberSpec] = None) -> D:

        return cls([((0, 2*math.pi), value)],
                   base_angle=base_angle) # type: ignore
    
    @property
    def base_angle(self) -> Fraction:
        return self._base_angle

    @property
    def int_dict(self) -> portion.IntervalDict:
        return portion.IntervalDict([
            (portion.closedopen(i*self._base_angle, (i+1)*self._base_angle), k)
                for i, k in enumerate(self._values)
        ])
    
    @property
    def bool_mask(self) -> 'BoolLidarMask':
        return self.map_bool(lambda x: x != self.default_value)

    @property
    def default_value(self) -> Optional[D]:
        return None
    
    @property
    def int_dict_sorted(self) -> List[Tuple[portion.Interval, D]]:
        return list(sorted(self.int_dict.as_dict(atomic=True).items(),
                           key=lambda x: x[0].lower))
    
    def map(self, f: Callable[[D], D]) -> 'LidarMask[D]':
        F = np.vectorize(f)
        return self.__class__(
            F(self._values),
            self.base_angle,
        )

    def map_poly(self, f: Callable[[D], K]) -> 'LidarMask[K]':
        F = np.vectorize(f)
        return LidarMask(
            F(self._values),
            self.base_angle,
        )

    def map_bool(self, f: Callable[[D], bool]) -> 'BoolLidarMask':
        return BoolLidarMask(self.map_poly(f))

    def __repr__(self) -> str:
        return f'{self.__class__.__name__}({repr(self._values), self._base_angle})'
    
    def __call__(self, t: float) -> D:
        return self.int_dict.get(t, self.default_value) # type: ignore
    
    def __eq__(self, other) -> 'BoolLidarMask':
        match other:
            case LidarMask():
                return self.zip_with(other, operator.eq) # type: ignore
            case _:
                return self.map_bool(partial(operator.eq, other))

    def __lt__(self, other : D) -> 'BoolLidarMask':
        return self.map_bool(partial(operator.gt, other)) # type: ignore

    def __le__(self, other : D) -> 'BoolLidarMask':
        # numpy-style pointwise comparisons
        return self.map_bool(partial(operator.ge, other)) # type: ignore
    
    def __gt__(self, other : D) -> 'BoolLidarMask':
        # numpy-style pointwise comparisons
        return self.map_bool(partial(operator.lt, other)) # type: ignore

    def __ge__(self, other : D) -> 'BoolLidarMask':
        # numpy-style pointwise comparisons
        return self.map_bool(partial(operator.le, other)) # type: ignore

    def zip_with(self, other: 'LidarMask[D]', f: Callable[[D, D], D]) -> 'LidarMask[D]':
        F = np.vectorize(f)
        assert self.base_angle == other.base_angle
        return self.__class__(
            F(self._values, other._values),
            self.base_angle,
        )

    def zip_with_poly(self, other: 'LidarMask[E]', f: Callable[[D, E], K]) -> 'LidarMask[K]':
        F = np.vectorize(f)
        assert self.base_angle == other.base_angle
        return LidarMask(
            F(self._values, other._values),
            self.base_angle,
        )
    
    def __add__(self, other: Union[D, 'LidarMask[D]']) -> 'LidarMask[D]':
        match other:
            case LidarMask():
                assert self.base_angle == other.base_angle
                return self.__class__(
                    self._values + other._values,
                    self.base_angle,
                )
            case _:
                return self.__class__(
                    self._values + other,
                    self.base_angle,
                )

    def __radd__(self, other: Union[D, 'LidarMask[D]']):
        return self.__class__(
            other + self._values,
            self.base_angle,
        )

    def __iadd__(self, other: Union[D, 'LidarMask[D]']):
        match other:
            case LidarMask():
                assert self.base_angle == other.base_angle
                self._values += other._values
            case _:
                self._values += other

        return self

    def __neg__(self):
        return self.map(operator.neg) # type: ignore

    def __sub__(self, other: Union[D, 'LidarMask[D]']) -> 'LidarMask[D]':
        match other:
            case LidarMask():
                assert self.base_angle == other.base_angle
                return self.__class__(
                    self._values - other._values,
                    self.base_angle,
                )
            case _:
                return self.__class__(
                    self._values - other,
                    self.base_angle,
                )

    def __rsub__(self, other: Union[D, 'LidarMask[D]']):
        return self.__class__(
            other - self._values,
            self.base_angle,
        )

    def __isub__(self, other: Union[D, 'LidarMask[D]']):
        match other:
            case LidarMask():
                assert self.base_angle == other.base_angle
                self._values -= other._values
            case _:
                self._values -= other

        return self


    def __mul__(self, other: Union[D, 'LidarMask[D]']) -> 'LidarMask[D]':
        match other:
            case LidarMask():
                assert self.base_angle == other.base_angle
                return self.__class__(
                    self._values * other._values,
                    self.base_angle,
                )
            case _:
                return self.__class__(
                    self._values * other,
                    self.base_angle,
                )

    def __rmul__(self, other: Union[D, 'LidarMask[D]']):
        return self.__class__(
            other * self._values,
            self.base_angle,
        )

    def __imul__(self, other: Union[D, 'LidarMask[D]']):
        match other:
            case LidarMask():
                assert self.base_angle == other.base_angle
                self._values *= other._values
            case _:
                self._values *= other

        return self
    
    def approx_eq(self, other):
        import pytest

        assert type(self) == type(other)

        assert self.base_angle == other.base_angle

        for x, y in zip(self._values, other._values):
            assert x == pytest.approx(y)


class BoolLidarMask(LidarMask[bool]):
    '''Boolean LIDAR data mask'''

    def __repr__(self) -> str:
        return f'{self.__class__.__name__}({repr(self.intervals)}, {self.base_angle})'

    @property
    def default_value(self):
        return False

    @property
    def intervals(self) -> List[portion.Interval]:
        return sum([list(k) for k,v in self.int_dict.items()
                    if v == self.default_value],
                   [])

    @property
    def prob_mask(self) -> 'ProbLidarMask':
        return ProbLidarMask(self.map_poly(lambda x: 1.0 if x else 0.0),
                             base_angle=self.base_angle)
    
    def pie_plot(self, **kwargs):
        from matplotlib import pyplot as plt
        
        slices = self.int_dict_sorted 

        fig = plt.figure(figsize=(2, 2))
        plt.pie(
            [k.upper - k.lower for k, _ in slices], # type: ignore
            colors=['white' if v else 'black'
                    for _, v in slices],
            wedgeprops=dict(edgecolor='black', antialiased=True, linewidth=2),
            startangle=90,
            counterclock=False,
        )
        return fig
    
    def plot(self, **kwargs):
        from matplotlib import pyplot as plt

        x = np.vectorize(float)(self.angles)
        y = self.prob_mask._values

        fig = plt.figure(figsize=(6, 1))
        plt.fill_between(x, y, color='black', **kwargs)
        plt.xlim(0, 2*math.pi)
        plt.xticks([i*math.pi / 4 for i in range(9)],
                   [r"$" + str(i) + r"\pi$/4" for i in range(9)])
        plt.yticks([0, 1], [0, 1])
        return fig
    

class ProbLidarMask(LidarMask[float]):
    '''Probablistic LIDAR mask'''
    @property
    def default_value(self) -> float:
        return 0.0

    def pie_plot(self, **kwargs):
        from matplotlib import pyplot as plt
        
        slices = self.int_dict_sorted 

        fig = plt.figure(figsize=(2, 2))
        plt.pie(
            [k.upper - k.lower for k, _ in slices], # type: ignore
            colors=[(1-v, 1-v, 1-v) for _, v in slices],
            wedgeprops=dict(edgecolor='black', antialiased=True, linewidth=2),
            startangle=90,
            counterclock=False,
        )
        return fig
    
    def plot(self, **kwargs):
        from matplotlib import pyplot as plt

        x = np.vectorize(float)(self.angles)
        y = self._values

        fig = plt.figure(figsize=(6, 1))
        plt.plot(x, y, color='black', **kwargs)
        plt.xlim(0, 2*math.pi)
        plt.xticks([i*math.pi / 4 for i in range(9)], [r"$" + str(i) + r"\pi$/4" for i in range(9)])
        plt.yticks([0, 1], [0, 1])
        return fig