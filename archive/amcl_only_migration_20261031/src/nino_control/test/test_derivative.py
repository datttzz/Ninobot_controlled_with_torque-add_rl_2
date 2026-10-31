"""Kiểm tra đạo hàm theo thời gian đo, chống kick và reset clock."""
import pytest
from nino_control.derivative import MeasurementDerivative


def test_variable_dt_filter_and_constant_measurement():
    derivative = MeasurementDerivative(0.05)
    assert derivative.update(1.0, 2.0) == 0
    assert derivative.update(1.1, 2.01) == pytest.approx(10/6)
    expected = (10/6) + (0.02/0.07)*(5-(10/6))
    assert derivative.update(1.2, 2.03) == pytest.approx(expected)
    # Đổi target không tác động đạo hàm vì helper chỉ nhận vận tốc đo.
    assert derivative.update(1.2, 2.03) == pytest.approx(expected)
    assert derivative.update(1.2, 2.04) == pytest.approx(expected*5/6)
    derivative.reset()
    assert derivative.update(8.0, 2.05) == 0


def test_clock_reset_and_missing_samples():
    derivative = MeasurementDerivative(0.05)
    derivative.update(0, 1)
    assert derivative.update(1, 1.01) > 0
    assert derivative.update(2, 0.5) == 0
    assert derivative.update(3, 2) == 0
    assert derivative.update(float('nan'), 2.01) == 0
    assert derivative.update(4, 2.02) == 0


@pytest.mark.parametrize('tau', [0, -1, float('nan')])
def test_reject_bad_filter(tau):
    with pytest.raises(ValueError):
        MeasurementDerivative(tau)
