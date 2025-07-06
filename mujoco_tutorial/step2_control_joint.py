import mujoco
import mujoco.viewer
import numpy as np
import time

# 1. scene.xml 경로
xml_path = "/home/sh0819/robo/mujoco_tutorial/scene.xml"

# 2. 모델 & 데이터 생성
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 3. 손 위치(body) ID 얻기
hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")

# 4. actuator 개수 확인
num_actuators = model.nu  # 보통 8 (7 arm + 1 gripper)
print(f"Number of actuators: {num_actuators}")

# 5. 목표 joint 위치 (7개 관절 + 1개 그리퍼 = 총 8개)
qpos_mid = np.array([0.0, -0.6, 0.0, -2.0, 0.0, 1.8, 0.6, 0.04])
qpos_final = np.array([0.0, -0.3, 0.0, -1.4, 0.0, 1.0, 0.6, 0.04])

# 6. Viewer 실행
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("🦾 Viewer 실행됨: 2단계로 로봇을 움직입니다...")

    # 1단계: 중간 위치로 이동
    for _ in range(300):  # 약 5초
        data.ctrl[:] = qpos_mid
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(1 / 60)

        pos = data.xpos[hand_id]
        print(f"[중간] Hand position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

    # 2단계: 최종 위치로 이동
    for _ in range(300):  # 약 5초
        data.ctrl[:] = qpos_final
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(1 / 60)

        pos = data.xpos[hand_id]
        print(f"[최종] Hand position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

    print("✅ 로봇 이동 완료.")
