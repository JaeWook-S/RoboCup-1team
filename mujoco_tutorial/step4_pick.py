import mujoco
import mujoco.viewer
import mujoco.glfw as glfw
import numpy as np
import time

# 로봇 환경 모델 로드
model = mujoco.MjModel.from_xml_path("/home/sh0819/week1/mujoco_tutorial/env.xml")
data = mujoco.MjData(model)

# 수동 뷰어 시작 (사용자가 카메라 조작 가능)
viewer = mujoco.viewer.launch_passive(model, data)

# gripper와 집을 대상 객체(cyl)의 ID를 가져옴
gripper_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripper")
box_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cyl")

# 초기 관절 각도 설정 (Franka Panda의 홈 자세)
initial_qpos = np.array([0.0, -0.4, 0.0, -2.0, 0.0, 1.8, 2.36])
data.qpos[:7] = initial_qpos
data.ctrl[:7] = initial_qpos

# 그리퍼 actuator 초기 상태 설정 (열림 상태: 255, 닫힘: 0)
gripper_close_init = 255
gripper_close_final = 0
data.ctrl[7] = gripper_close_init
mujoco.mj_forward(model, data)  # 모델과 데이터를 동기화

# 두 pose 사이 보간 함수 (부드럽게 이동하기 위해 사용)
def interpolate_qpos(start, target, alpha):
    return (1 - alpha) * np.array(start) + alpha * np.array(target)

# 간단한 inverse kinematics 함수 (위치 + 방향 제어)
def ik_with_orientation(target_pos, target_z, gain=0.3):
    for _ in range(20):
        mujoco.mj_forward(model, data)

        # 현재 gripper 위치 및 방향 추출
        current_pos = data.site_xpos[gripper_site_id]
        current_xmat = data.site_xmat[gripper_site_id].reshape(3, 3)
        current_z = current_xmat[:, 2]  # Z축 방향 추출

        # 위치 오차 및 방향 오차 계산
        pos_error = target_pos - current_pos
        ori_error = np.cross(current_z, target_z)

        # 오차가 작으면 종료
        if np.linalg.norm(pos_error) < 1e-3 and np.linalg.norm(ori_error) < 1e-2:
            break

        # 위치 및 회전 자코비안 계산
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, gripper_site_id)
        J = np.vstack([jacp[:, :7], jacr[:, :7]])  # 6x7 자코비안

        # 오차 기반 joint delta 계산
        error = np.concatenate([gain * pos_error, 0.2 * ori_error])
        dq = np.linalg.pinv(J) @ error

        # 관절 업데이트 및 제어 신호 적용
        data.qpos[:7] += dq
        data.ctrl[:7] = data.qpos[:7]
        mujoco.mj_forward(model, data)

# 상자 위치 가져오기 및 타겟 위치 계산
mujoco.mj_forward(model, data)
box_pos = data.xpos[box_body_id]
grip_above = box_pos + np.array([0, 0, 0.15])   # 위에서 접근
grip_down = box_pos + np.array([0, 0, 0.075])   # 가까이 접근
target_z = np.array([0, 0, -1])  # 그리퍼의 z축 방향을 아래로

# 이동 시간 및 스텝 수 설정
move_duration = 10.0
n_steps = int(move_duration / model.opt.timestep)

# 1단계: 현재 위치에서 grip_above 위치로 이동
for step in range(n_steps):
    alpha = step / n_steps
    current_pos = data.site_xpos[gripper_site_id]
    target_pos = (1 - alpha) * current_pos + alpha * grip_above
    ik_with_orientation(target_pos, target_z)   # IK로 위치 조정
    data.qpos[7] = 0.04  # 그리퍼 열기 유지
    viewer.sync()
    mujoco.mj_step(model, data)

# 2단계: grip_above → grip_down 위치로 이동 (내려가기)
for step in range(n_steps):
    alpha = step / n_steps
    target_pos = (1 - alpha) * grip_above + alpha * grip_down
    ik_with_orientation(target_pos, target_z)
    data.qpos[7] = 0.04  # 계속 열려 있음
    viewer.sync()
    mujoco.mj_step(model, data)

# 3단계: 그리퍼 천천히 닫기
ctrl = data.ctrl.copy()
finger_pos = ctrl[7] 

for i in range(255):
    data.ctrl = ctrl                       # 제어 신호 적용
    mujoco.mj_step(model, data)           # 한 스텝 실행
    finger_pos -= 1                       # 그리퍼 닫기 (값 감소)
    ctrl[7] = max(finger_pos, 0)          # 0보다 작지 않도록 제한
    viewer.sync()
    time.sleep(0.01)                      # 천천히 닫히게 sleep

# 4단계: 4번째 관절을 천천히 펴는 동작
joint_4_pos = ctrl[3]  # 초기 위치 저장

for i in range(500):
    data.ctrl = ctrl
    mujoco.mj_step(model, data, nstep=1)
    joint_4_pos += 0.003               # 조금씩 증가시켜 펴기
    ctrl[3] = joint_4_pos
    viewer.sync()
    time.sleep(0.01)

# 시뮬레이션이 끝난 후에도 계속 실행되도록 루프 유지
while viewer.is_running():
    mujoco.mj_step(model, data)
    viewer.sync()
    time.sleep(0.01)