import numpy as np
from filtre.angles import wrap
from filtre.kalman import KalmanFilter
from filtre.filter_node import FilterNode
from interfaces import PoseMeasurement, ControlInput

R = np.diag([2.0 ** 2, 2.0 ** 2, 0.02 ** 2])
Q_FLOOR = np.diag([0.1, 0.1, 1e-4])


def make_kf(z0=(100.0, 100.0, 0.0), R=R):
    return KalmanFilter(z0, R, sigma_v=5.0, sigma_w=0.05, q_floor=Q_FLOOR)


def test_wrap_range():
    for a in [0, 1, np.pi + 0.1, -np.pi - 0.1, 7.0, -7.0]:
        assert -np.pi <= wrap(a) <= np.pi


def test_translation_straight():
    kf = make_kf(z0=(0, 0, 0))
    kf.predict(v=100.0, w=0.0, dt=0.1)
    assert np.allclose(kf.x, [10.0, 0.0, 0.0])


def test_rotation_in_place():
    kf = make_kf(z0=(50, 50, 0))
    kf.predict(v=0.0, w=1.0, dt=0.1)
    assert np.allclose(kf.x[:2], [50, 50]) and np.isclose(kf.x[2], 0.1)


def test_P_grows_without_measurement():
    kf = make_kf()
    traces = []
    for _ in range(20):
        kf.predict(50.0, 0.0, 0.1)
        traces.append(np.trace(kf.P))
    assert all(b > a for a, b in zip(traces, traces[1:]))


def test_P_shrinks_after_update():
    kf = make_kf()
    for _ in range(10):
        kf.predict(50.0, 0.0, 0.1)
    tr_before = np.trace(kf.P)
    kf.update(kf.x.copy())
    assert np.trace(kf.P) < tr_before


def test_angle_innovation_wraps():
    # 178 deg estime, 182 deg (= -178 deg) mesure : l'ecart reel est ~4 deg
    kf = make_kf(z0=(0, 0, np.deg2rad(178)))
    innov, _, _ = kf.update(np.array([0, 0, np.deg2rad(-178)]))
    assert abs(innov[2]) < np.deg2rad(10)
    assert abs(abs(kf.x[2]) - np.pi) < np.deg2rad(10)  # reste pres de +-pi


def test_huge_R_keeps_prediction():
    kf = make_kf()
    kf.R = np.diag([1e12, 1e12, 1e12])   # camera tres mauvaise (apres initialisation)
    kf.predict(100.0, 0.0, 0.1)
    x_pred = kf.x.copy()
    kf.update(np.array([500.0, 500.0, 1.0]))
    assert np.allclose(kf.x, x_pred, atol=1e-3)


def test_P_stays_symmetric():
    kf = make_kf()
    for _ in range(200):
        kf.predict(50.0, 0.1, 0.1)
        kf.update(kf.x + np.array([1.0, -1.0, 0.01]))
    assert np.allclose(kf.P, kf.P.T, atol=1e-9)
    assert np.all(np.linalg.eigvalsh(kf.P) > 0)


def test_node_survives_hidden_qr():
    node = FilterNode()
    node.on_measurement(PoseMeasurement(t=0.0, visible=True, x=0.0, y=0.0, theta=0.0))
    est = node.step(0.0)
    assert est is not None and est.measurement_used
    node.on_control(ControlInput(t=0.0, v=100.0, omega=0.0))
    for k in range(1, 11):                       # QR cache : aucune mesure
        node.on_measurement(PoseMeasurement(t=k * 0.1, visible=False))
        est = node.step(k * 0.1)
        assert not est.measurement_used
    assert est.x > 90.0                          # ~100 mm parcourus, par prediction seule
