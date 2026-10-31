import numpy as np
import pytest
from rl_training.localization_math import compose,inverse,interpolate_pose,aligned_errors


def test_transform_same_point_inverse_and_decomposition():
    correction=np.array([.3,-.2,.4]);odom=np.array([1.,.2,-.1])
    mapped=compose(correction,odom)
    assert compose(inverse(correction),mapped)==pytest.approx(odom)
    assert compose(mapped,inverse(odom))==pytest.approx(correction)


def test_wrap_and_common_time_without_extrapolation():
    a=np.array([[0,0,0,np.deg2rad(179)],[1,1,0,np.deg2rad(-179)]])
    out=interpolate_pose(a,[-.1,.5,1.1],max_gap=2)
    assert np.isnan(out[[0,2]]).all()
    assert abs(out[1,2])==pytest.approx(np.pi)
    g=np.array([[.2,.2,0,0],[.5,.5,0,0],[.8,.8,0,0]])
    t,_,_,error=aligned_errors(a,g,[0,0,0])
    assert not len(t) # Không nội suy qua gap 1 giây như cảm biến bình thường.


def test_known_dynamic_error_and_static_registration():
    gt=np.array([[0,0,0,0],[.05,1,0,.1],[.1,2,0,.2]])
    T=[.2,-.1,.3];amcl=np.column_stack([gt[:,0],compose(inverse(T),gt[:,1:])])
    _,_,_,e=aligned_errors(amcl,gt,T)
    assert e==pytest.approx(np.zeros_like(e),abs=1e-14)
    # Một transform tĩnh không được phép xóa drift thay đổi theo thời gian.
    amcl[-1,1]+=.5
    _,_,_,e=aligned_errors(amcl,gt,T)
    assert np.linalg.norm(e[-1,:2])==pytest.approx(.5)


def test_duplicate_and_backwards_stamps():
    data=np.array([[0,0,0,0],[.05,1,0,0],[.05,2,0,0],[.1,3,0,0]])
    assert interpolate_pose(data,[.05])[0,0]==2
    with pytest.raises(ValueError):interpolate_pose(data[::-1],[.05])


def test_map_field_half_cell_border_is_inside():
    from rl_training.localization_diagnosis import MapField
    field=MapField.__new__(MapField)
    field.origin=np.array([0.,0.]);field.res=1.;field.h=field.w=2
    field.dist=np.zeros((2,2))
    assert field.distances(np.array([[.1,.1],[1.9,1.9],[-.01,0],[2.,1.]])).tolist()==[0.,0.,2.,2.]
