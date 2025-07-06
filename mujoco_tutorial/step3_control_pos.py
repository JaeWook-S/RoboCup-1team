import mujoco
import mujoco.viewer
import numpy as np
import time

# 역기구학 함수 (위치 + 방향 제어)
def ik_with_orientation(model, data, target_pos, target_z, site_id, gain=0.3):
    for _ in range(50):  # 더 정밀하게
        mujoco.mj_forward(model, data)
        current_pos = data.site_xpos[site_id]
        current_xmat = data.site_xmat[site_id].reshape(3, 3)
        current_z = current_xmat[:, 2]

        pos_error = target_pos - current_pos
        ori_error = np.cross(current_z, target_z)

        if np.linalg.norm(pos_error) < 1e-3 and np.linalg.norm(ori_error) < 1e-2:
            break

        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        J = np.vstack([jacp[:, :7], jacr[:, :7]])

        error = np.concatenate([gain * pos_error, 0.2 * ori_error])
        dq = np.linalg.pinv(J) @ error

        data.qpos[:7] += dq
        data.ctrl[:7] = data.qpos[:7]
        mujoco.mj_forward(model, data)

# 두 pose 사이 보간 함수
def interpolate_qpos(start, target, alpha):
    return (1 - alpha) * np.array(start) + alpha * np.array(target)

# =============================
# 🔧 모델 초기화
# =============================
model = mujoco.MjModel.from_xml_path("/home/sh0819/robo/mujoco_tutorial/env.xml")
data = mujoco.MjData(model)

# 이름으로 ID 얻기
box_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box")
gripper_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripper")

# 초기 관절 자세
initial_qpos = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.8, 2.355])
data.qpos[:7] = initial_qpos
mujoco.mj_forward(model, data)

# =============================
# 🎯 목표 위치 설정 및 IK 수행
# =============================
target_pos = data.xpos[box_body_id].copy() + np.array([0, 0, 0.2])  # 더 높은 위치
target_z = np.array([0, 0, -1])  # 아래 방향

# 역기구학 수행하여 목표 관절값 얻기
ik_with_orientation(model, data, target_pos, target_z, gripper_site_id)
target_qpos = data.qpos[:7].copy()  # IK 결과 저장

# =============================
# 🎞 이동 계획 설정
# =============================
move_duration = 5  # 더 천천히 이동
n_steps = int(move_duration / model.opt.timestep)
start_qpos = initial_qpos.copy()

# =============================
# 🖼 Viewer를 통한 실행
# =============================
with mujoco.viewer.launch_passive(model, data) as viewer:
    for step in range(n_steps):
        alpha = step / n_steps
        desired_qpos = interpolate_qpos(start_qpos, target_qpos, alpha)

        data.qpos[:7] = desired_qpos
        data.qpos[7] = 0.04  # 그리퍼 열기
        mujoco.mj_forward(model, data)
        viewer.sync()
        mujoco.mj_step(model, data)
    print("✅ 이동 완료!")

    # 종료까지 대기
    while viewer.is_running():
        viewer.sync()
