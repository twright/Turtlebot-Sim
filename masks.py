# LIDAR masking

# import typing
from typing import Union, TypeVar, Tuple, Generic, List, Callable, Iterable, Type

D = TypeVar('D')
K = TypeVar('K')
E = TypeVar('E')

from abc import ABCMeta
import math
import operator
from functools import partial
from collections.abc import Sequence
from collections import deque

import portion
from portion import IntervalDict
from portion.interval import Interval
from portion.interval import Bound

# TODO: do we need any special handing of the final endpoint?
DOMAIN = Interval.from_atomic(Bound.CLOSED, 0, 2*math.pi, Bound.OPEN)

def interval_from_tuple(x: Union[Tuple[float, float], IntervalDict]):
    match x:
        case (l, u):
            return Interval.from_atomic(
                Bound.CLOSED,
                l,
                u,
                Bound.CLOSED,
            )
        case Interval():
            return x
        case _:
            raise TypeError('Not a valid interval-like object')


class LidarMask[D]:
    '''Signal of type [0, 2pi] -> D representing a Lidar occlusion mask.'''
    # Keys should be type T
    _intervals: IntervalDict
    
    def __init__(self, contents_spec):
        # Special case handling of boolean input
        if isinstance(contents_spec, Sequence) and len(contents_spec) > 0:
            match contents_spec[0]:
                case ((_, _), _):
                    pass
                case (Interval(), _):
                    pass
                case _:
                    # plain interval mask
                    contents_spec = [(x, True) for x in contents_spec]

        if isinstance(contents_spec, LidarMask):
            contents_spec = contents_spec.int_dict

        if not isinstance(contents_spec, IntervalDict):
            contents_spec = IntervalDict([
                (interval_from_tuple(k), v) for k,v in contents_spec
            ])

        contents_spec = IntervalDict([
            (k.intersection(DOMAIN), v) for k,v in contents_spec.items()
        ])

        self._int_dict = contents_spec

    @property
    def completion(self):
        return self.__class__(
            self.int_dict.combine(
                IntervalDict([(DOMAIN, self.default_value)]),
                (lambda x, _: x),
            )
        ) 

    @classmethod
    def total_mask(cls : Type[K], value : D) -> K:
        return cls([(DOMAIN, value)]) # type: ignore

    @property
    def int_dict(self) -> IntervalDict:
        return self._int_dict
    
    @property
    def bool_mask(self) -> 'BoolLidarMask':
        return self.map_bool(lambda x: x != self.default_value)

    @property
    def default_value(self):
        return None
    
    def map(self, f: Callable[[D], D]) -> 'LidarMask[D]':
        return self.__class__(
            IntervalDict([
                (k, f(v)) for k,v in self.int_dict.items()
            ])
        )

    def map_poly(self, f: Callable[[D], K]) -> 'LidarMask[K]':
        return LidarMask(
            IntervalDict([
                (k, f(v)) for k,v in self.int_dict.items()
            ])
        )

    def map_bool(self, f: Callable[[D], bool]) -> 'BoolLidarMask':
        return BoolLidarMask(self.map_poly(f))

    def __repr__(self) -> str:
        return f'{self.__class__.__name__}({repr(list(self.int_dict.as_dict(atomic=True).items()))})'
    
    def __call__(self, t: float) -> D:
        return self.int_dict.get(t, self.default_value) # type: ignore
    
    def __eq__(self, other) -> Union[bool, 'BoolLidarMask']:
        match other:
            case LidarMask():
                return self.int_dict == other.int_dict
            case _:
                # numpy-style pointwise comparisons
                return BoolLidarMask(self.int_dict.combine(
                    other.int_dict, operator.eq,
                ))

    def __lt__(self, other : D) -> 'BoolLidarMask':
        # numpy-style pointwise comparisons
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
        return self.__class__(self.int_dict.combine(other.int_dict, f))

    def zip_with_poly(self, other: 'LidarMask[E]', f: Callable[[D, E], K]) -> 'LidarMask[K]':
        return LidarMask(self.int_dict.combine(other.int_dict, f))
    
    def __add__(self, other: Union[D, 'LidarMask[D]']) -> 'LidarMask[D]':
        return self.completion.zip_with(
            (other if isinstance(other, LidarMask)
                   else self.__class__.total_mask(other)),
            operator.add,
        )

    def __radd__(self, other: Union[D, 'LidarMask[D]']):
        return self.__class__(
            (other if isinstance(other, LidarMask)
                   else self.__class__.total_mask(other))
            + self
        )

    def __neg__(self):
        return self.map(lambda x: -x) # type: ignore

    def __sub__(self, other: Union[D, 'LidarMask[D]']):
        return self + (-other) # type: ignore

    def __rsub__(self, other: Union[D, 'LidarMask[D]']):
        return other + (-self)

    def __mul__(self, other: Union[D, 'LidarMask[D]']):
        return self.completion.zip_with(
            (other if isinstance(other, LidarMask)
                   else self.__class__.total_mask(other)),
            operator.mul,
        )

    def __rmul__(self, other: Union[D, 'LidarMask[D]']):
        return self.__class__(
            (other if isinstance(other, LidarMask)
                   else self.__class__.total_mask(other))
            * self
        )

    def __div__(self, other: Union[D, 'LidarMask[D]']):
        return self.zip_with(
            (other if isinstance(other, LidarMask)
                   else self.__class__.total_mask(other)),
            operator.truediv,
        )
    
    def approx_eq(self, other: 'LidarMask[D]'):
        import pytest

        for x in portion.iterate(DOMAIN, math.pi/32):
            assert self(x) == pytest.approx(other(x))


class BoolLidarMask(LidarMask[bool]):
    '''Boolean LIDAR data mask'''
    def __init__(self, contents_spec):
        super().__init__(contents_spec)

        # Only keep positive elements to normalize
        self._int_dict = IntervalDict(
            (k,v) for k,v in self._int_dict.items() if v 
        )

    def __repr__(self) -> str:
        return f'{self.__class__.__name__}({repr(self.intervals)})'

    @property
    def default_value(self):
        return False

    @property
    def intervals(self) -> List[Interval]:
        return sum(map(list, self.int_dict), [])
    

class ProbLidarMask(LidarMask[float]):
    '''Probablistic LIDAR mask'''
    def __init__(self, contents_spec):
        super().__init__(contents_spec)

        self._int_dict = IntervalDict(
            (k, 0.0 if v is None else v)
                for k,v in self._int_dict.items()
        )

    @property
    def default_value(self) -> float:
        return 0.0
