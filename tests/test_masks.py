import pytest
from masks import BoolLidarMask, LidarMask

class TestLidarMask:
    I1 = [((0, 1), 0.7), ((2, 3), 0.7)]
    B1 = [(0, 1), (2, 3)]
    I2 = [((0, 1), 0.4), ((2, 3), 0.7)]

    def test_repr(self):
        assert repr(LidarMask(self.I1)) == 'LidarMask([([0,1], 0.7), ([2,3], 0.7)])'

    def test_boolean_mask(self):
        assert LidarMask(self.I1).bool_mask == BoolLidarMask(self.B1)

    def test_comparision_1(self):
        assert (LidarMask(self.I1) < 0.8) == LidarMask(self.I1).bool_mask

    def test_comparision_2(self):
        assert (LidarMask(self.I1) < 0.4) == BoolLidarMask([])

    def test_comparision_3(self):
        assert (LidarMask(self.I2) < 0.5) == BoolLidarMask([(0, 1)])

    def test_arithmetic(self):
        assert (LidarMask(self.I1) + LidarMask(self.I2)).completion \
            == LidarMask([((0,1), 1.1), ((2,3), 1.4)]).completion