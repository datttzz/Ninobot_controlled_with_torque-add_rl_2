import numpy as np
import pytest
from rl_training.localization_math import compose,inverse,interpolate_pose


def test_se2_inverse_preserves_same_point():
    correction=np.array([.3,-.2,.4]);odom=np.array([1.,.2,-.1])
    assert compose(inverse(correction),compose(correction,odom))==pytest.approx(odom)


def test_wrap_gap_and_no_extrapolation():
    data=np.array([[0,0,0,np.deg2rad(179)],[.1,1,0,np.deg2rad(-179)]])
    values=interpolate_pose(data,[-.1,.05,.2])
    assert np.isnan(values[[0,2]]).all()
    assert abs(values[1,2])==pytest.approx(np.pi)
    data[-1,0]=1.
    assert np.isnan(interpolate_pose(data,[.5])).all()


def test_duplicate_and_backwards_stamps():
    data=np.array([[0,0,0,0],[.05,1,0,0],[.05,2,0,0],[.1,3,0,0]])
    assert interpolate_pose(data,[.05])[0,0]==2
    with pytest.raises(ValueError):interpolate_pose(data[::-1],[.05])
