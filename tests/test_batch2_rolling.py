import mujoco
import numpy as np
import pytest

from wbr.sim.rolling import RollingLoss

# Independently chosen primitive fixture. Geometry and inertial values are synthetic.
COAST_XML = """
<mujoco>
  <option timestep="{step}" gravity="0 0 -9.81"/>
  <default><geom friction="1 0 0" condim="3"/></default>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 .1"/>
    <body name="carrier" pos="0 0 .1">
      <joint name="x" type="slide" axis="1 0 0"/>
      <joint name="z" type="slide" axis="0 0 1"/>
      <joint name="pitch" type="hinge" axis="0 1 0" stiffness="2" damping=".1"/>
      <inertial pos="0 0 0" mass="2" diaginertia=".04 .04 .04"/>
      <body name="wheel">
        <joint name="spin" type="hinge" axis="0 1 0"/>
        <geom name="tire" type="cylinder" size=".1 .02" zaxis="0 1 0" mass=".3"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def fixture(step):
    model = mujoco.MjModel.from_xml_string(COAST_XML.format(step=step))
    return model, mujoco.MjData(model)


def coast(step, sign, coefficient):
    model, data = fixture(step)
    spin = model.joint("spin").id
    wheel = model.body("wheel").id
    surface_pair = {model.geom("floor").id, model.geom("tire").id}
    loss = RollingLoss(model, data, [spin], 0.1, coefficient, 0.2)
    for _ in range(round(0.3 / step)):
        mujoco.mj_step(model, data)
    data.qvel[0] = sign * 0.45
    data.qvel[3] = sign * 4.5

    energy = 0.0
    wrench = np.zeros(6)
    for _ in range(round(1.0 / step)):
        with np.errstate(all="raise"):
            loss.clear()
            mujoco.mj_forward(model, data)
            load = 0.0
            for index in range(data.ncon):
                contact = data.contact[index]
                if {contact.geom1, contact.geom2} == surface_pair:
                    mujoco.mj_contactForce(model, data, index, wrench)
                    load += max(0.0, wrench[0])
            loss.apply([load])
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, wheel, wrench, 0)
            power = data.xfrc_applied[wheel, 3:] @ wrench[:3]
            assert power <= 1e-12
            assert not np.any(data.xfrc_applied[model.body("carrier").id])
            assert not np.any(data.qfrc_applied)
            energy -= power * step
            mujoco.mj_step(model, data)
    return sign * data.qvel[0], energy


@pytest.mark.parametrize("sign", [-1, 1])
def test_loaded_coast_down_slows_in_both_directions_at_two_steps(sign):
    unresisted, no_energy = coast(0.002, sign, 0.0)
    coarse, coarse_energy = coast(0.002, sign, 0.075)
    fine, fine_energy = coast(0.001, sign, 0.075)
    assert unresisted > 0.4 and no_energy == 0.0
    assert 0.0 < coarse < 0.05 and 0.0 < fine < 0.05
    assert abs(coarse - fine) < 0.015
    assert coarse_energy > 0.15 and fine_energy > 0.15


def test_world_spin_includes_carrier_pitch_and_unloaded_torque_clears():
    model, data = fixture(0.001)
    joint = model.joint("spin").id
    wheel = model.body("wheel").id
    loss = RollingLoss(model, data, [joint], 0.1, 0.05, 0.2)
    data.qvel[2] = 2.0
    data.xfrc_applied[wheel, 0] = 3.0
    mujoco.mj_forward(model, data)
    loss.apply([4.0])
    assert data.qvel[3] == 0.0
    np.testing.assert_allclose(data.xfrc_applied[wheel, 4], -0.02 * np.tanh(10.0))
    loss.clear()
    np.testing.assert_array_equal(data.xfrc_applied[wheel], [3, 0, 0, 0, 0, 0])
    loss.apply([0.0])
    assert not np.any(data.xfrc_applied[wheel, 3:])
    with pytest.raises(ValueError, match="normal load"):
        loss.apply([-1.0])
