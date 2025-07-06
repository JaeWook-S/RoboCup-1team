import mujoco
import mujoco.viewer
import time

# 1. XML 경로
xml_path = "/home/sh0819/robo/mujoco_tutorial/env.xml"

# 2. 모델 및 데이터 로딩
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 3. 손 (end-effector) body ID
hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")

# 4. Viewer 실행 (기본 timestep으로 시뮬레이션 진행)
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("🦾 MuJoCo Viewer 실행 중...\n")

    while viewer.is_running():
        mujoco.mj_step(model, data)   # 물리 시뮬레이션 1 스텝
        viewer.sync()                 # 화면 렌더링 동기화
        time.sleep(model.opt.timestep)  # 기본 timestep만큼 대기

        # 손 위치 출력
        pos = data.xpos[hand_id]
        print(f"🔹 Hand 위치: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
