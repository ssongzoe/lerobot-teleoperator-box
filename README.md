# LeRobot Teleoperator Box

Meta Quest VR을 사용하여 RB-Y1의 양팔, 토르소 및 모바일 베이스를 제어하고 LeRobot 데이터셋을 기록하기 위한 Teleoperator 패키지입니다.

지원하는 제어 대상은 다음과 같습니다.

* 오른팔 Cartesian End-Effector pose
* 왼팔 Cartesian End-Effector pose
* 토르소 Cartesian pose
* 모바일 베이스 SE(2) 속도
* 그리퍼 제어 예정

---

# 1. Installation

LeRobot 가상환경을 활성화합니다.

```bash
cd ~/rby1-lerobot
source .venv/bin/activate
```

패키지 디렉터리에서 editable mode로 설치합니다.

```bash
cd ~/project/rby1-lerobot/lerobot-teleoperator-box
python -m pip install -e .
```

설치 여부를 확인합니다.

```bash
python -m pip show lerobot-teleoperator-box
```

Python import를 확인합니다.

```bash
python -c "from lerobot_teleoperator_box import BoxVr, BoxVrConfig; print('IMPORT OK')"
```

PC와 Meta Quest는 동일한 네트워크에 연결되어 있어야 합니다.

* `teleop.local_ip`: LeRobot을 실행하는 PC의 IP
* `teleop.local_port`: Quest controller pose를 수신할 UDP 포트
* `teleop.meta_quest_ip`: Meta Quest의 IP
* `teleop.meta_quest_port`: Quest handshake 수신 포트

PC의 IP는 다음 명령어로 확인할 수 있습니다.

```bash
hostname -I
```

Meta Quest 앱을 실행한 후 양쪽 컨트롤러의 아무 버튼이나 한 번 눌러 controller pose streaming을 활성화해야 합니다.

---

# 2. Dual-Arm Teleoperate

Meta Quest의 양쪽 컨트롤러를 사용하여 RB-Y1의 양팔을 Cartesian 방식으로 제어합니다.

토르소와 모바일 베이스는 사용하지 않습니다.

```bash
lerobot-teleoperate \
  --robot.type=rby1 \
  --robot.address=192.168.30.1:50051 \
  --robot.model=m \
  --robot.action_mode=ee \
  --robot.use_torso=false \
  --robot.use_right_arm=true \
  --robot.use_left_arm=true \
  --robot.use_mobile_base=false \
  --robot.use_gripper=false \
  --robot.ee_whole_body=false \
  --teleop.type=box_vr \
  --teleop.local_ip=192.168.0.132 \
  --teleop.local_port=5005 \
  --teleop.meta_quest_ip=192.168.0.69 \
  --teleop.meta_quest_port=6000 \
  --teleop.send_handshake=true \
  --teleop.use_torso=false \
  --teleop.use_right_arm=true \
  --teleop.use_left_arm=true \
  --teleop.use_mobile_base=false \
  --teleop.use_gripper=false
```

## Controls

* **Right grip 유지:** 오른팔 Cartesian pose 추종
* **Left grip 유지:** 왼팔 Cartesian pose 추종
* **Grip 해제:** 해당 팔의 마지막 Cartesian target 유지
* **Trigger:** gripper control 예정

---

# 3. Whole-Body Teleoperate

Meta Quest VR을 사용하여 RB-Y1의 토르소, 모바일 베이스 및 양팔을 동시에 제어합니다.

```bash
lerobot-teleoperate \
  --robot.type=rby1 \
  --robot.address=192.168.30.1:50051 \
  --robot.model=m \
  --robot.action_mode=ee \
  --robot.use_torso=true \
  --robot.use_right_arm=true \
  --robot.use_left_arm=true \
  --robot.use_mobile_base=true \
  --robot.use_gripper=false \
  --robot.ee_whole_body=false \
  --teleop.type=box_vr \
  --teleop.local_ip=192.168.0.132 \
  --teleop.local_port=5005 \
  --teleop.meta_quest_ip=192.168.0.69 \
  --teleop.meta_quest_port=6000 \
  --teleop.send_handshake=true \
  --teleop.use_torso=true \
  --teleop.use_right_arm=true \
  --teleop.use_left_arm=true \
  --teleop.use_mobile_base=true \
  --teleop.use_gripper=false
```

## Controls

- **Right grip 유지:** 오른팔 Cartesian pose 추종
- **Left grip 유지:** 왼팔 Cartesian pose 추종
- **양쪽 grip 동시 유지:** 양팔 동시 추종
- **Left Y 버튼 유지:** HMD 움직임을 이용한 토르소 Cartesian pose 추종
- **Grip 또는 Y 버튼 해제:** 해당 구성요소의 마지막 Cartesian target 유지
- **Right thumbstick 위·아래:** 모바일 베이스 전진·후진
- **Right thumbstick 좌·우:** 모바일 베이스 좌·우 횡이동
- **Left thumbstick 좌·우:** 모바일 베이스 회전
- **Trigger:** gripper control 예정

토르소는 양쪽 grip을 동시에 누르는 순간 현재 HMD pose와 현재 토르소 pose를 기준점으로 설정합니다.

이후 HMD의 상하 이동과 회전이 토르소 Cartesian target에 반영됩니다. HMD의 전후 및 좌우 translation은 토르소 target에 반영하지 않습니다.

최초 물리 테스트에서는 다음 값을 권장합니다.

```bash
--teleop.torso_position_scale=0.5 \
--teleop.torso_rotation_scale=0.5
```

움직임 방향과 범위를 확인한 뒤 필요에 따라 `1.0`까지 높일 수 있습니다.

## Cartesian Solver

기본 설정은 다음과 같습니다.

```bash
--robot.ee_whole_body=false
```

이 설정에서는 토르소, 오른팔, 왼팔이 각각 별도의 Cartesian impedance solver를 사용합니다.

모바일 베이스의 SE(2) 속도 명령은 `ee_whole_body` 설정과 관계없이 별도로 실행됩니다.

---

# 4. Dual-Arm Record

Meta Quest를 사용하여 RB-Y1 양팔 Cartesian teleoperation 데이터를 기록합니다.

토르소와 모바일 베이스는 기록하지 않습니다.

```bash
lerobot-record \
  --robot.type=rby1 \
  --robot.address=192.168.30.1:50051 \
  --robot.model=m \
  --robot.action_mode=ee \
  --robot.use_torso=false \
  --robot.use_right_arm=true \
  --robot.use_left_arm=true \
  --robot.use_mobile_base=false \
  --robot.use_gripper=false \
  --robot.ee_whole_body=false \
  --teleop.type=box_vr \
  --teleop.local_ip=192.168.0.132 \
  --teleop.local_port=5005 \
  --teleop.meta_quest_ip=192.168.0.69 \
  --teleop.meta_quest_port=6000 \
  --teleop.send_handshake=true \
  --teleop.use_torso=false \
  --teleop.use_right_arm=true \
  --teleop.use_left_arm=true \
  --teleop.use_mobile_base=false \
  --teleop.use_gripper=false \
  --dataset.repo_id=<hf_username>/<dataset_name> \
  --dataset.single_task="Control both arms of RB-Y1 using Meta Quest VR." \
  --dataset.num_episodes=50 \
  --dataset.fps=30
```

예시:

```bash
--dataset.repo_id=rainbowrobotics/rby1_dual_arm_vr
```

기록되는 주요 action은 다음과 같습니다.

```text
right_ee.x
right_ee.y
right_ee.z
right_ee.wx
right_ee.wy
right_ee.wz

left_ee.x
left_ee.y
left_ee.z
left_ee.wx
left_ee.wy
left_ee.wz
```

---

# 5. Whole-Body Record

Meta Quest를 사용하여 RB-Y1의 토르소, 모바일 베이스 및 양팔 teleoperation 데이터를 함께 기록합니다.

```bash
lerobot-record \
  --robot.type=rby1 \
  --robot.address=192.168.30.1:50051 \
  --robot.model=m \
  --robot.action_mode=ee \
  --robot.use_torso=true \
  --robot.use_right_arm=true \
  --robot.use_left_arm=true \
  --robot.use_mobile_base=true \
  --robot.use_gripper=false \
  --robot.ee_whole_body=false \
  --teleop.type=box_vr \
  --teleop.local_ip=192.168.0.132 \
  --teleop.local_port=5005 \
  --teleop.meta_quest_ip=192.168.0.69 \
  --teleop.meta_quest_port=6000 \
  --teleop.send_handshake=true \
  --teleop.use_torso=true \
  --teleop.use_right_arm=true \
  --teleop.use_left_arm=true \
  --teleop.use_mobile_base=true \
  --teleop.use_gripper=false \
  --teleop.torso_position_scale=0.5 \
  --teleop.torso_rotation_scale=0.5 \
  --dataset.repo_id=<hf_username>/<dataset_name> \
  --dataset.single_task="Control the torso, mobile base, and both arms of RB-Y1 using Meta Quest VR." \
  --dataset.num_episodes=50 \
  --dataset.fps=30
```

예시:

```bash
--dataset.repo_id=rainbowrobotics/rby1_whole_body_vr
```

기록되는 주요 action은 다음과 같습니다.

```text
torso_ee.x
torso_ee.y
torso_ee.z
torso_ee.wx
torso_ee.wy
torso_ee.wz

right_ee.x
right_ee.y
right_ee.z
right_ee.wx
right_ee.wy
right_ee.wz

left_ee.x
left_ee.y
left_ee.z
left_ee.wx
left_ee.wy
left_ee.wz

x.vel
y.vel
theta.vel
```

## Dataset Options

* `dataset.repo_id`: Hugging Face dataset repository
* `dataset.single_task`: 모든 episode에 적용할 task instruction
* `dataset.num_episodes`: 기록할 episode 수
* `dataset.fps`: 데이터셋 저장 FPS

토르소와 모바일 베이스 action을 기록하려면 robot과 teleoperator 양쪽에서 관련 옵션을 모두 활성화해야 합니다.

```bash
--robot.use_torso=true
--teleop.use_torso=true

--robot.use_mobile_base=true
--teleop.use_mobile_base=true
```

---

# Notes

* Meta Quest 앱 실행 후 양쪽 컨트롤러의 아무 버튼이나 눌러 controller tracking을 활성화해야 합니다.
* `teleop.local_ip`에 `0.0.0.0`이 아닌 실제 PC 네트워크 IP를 입력해야 합니다.
* `robot.action_mode=ee`가 설정되어 있어야 Cartesian action을 사용할 수 있습니다.
* 양팔만 사용할 때는 torso와 mobile base 옵션을 모두 `false`로 설정합니다.
* 전신 제어 시 최초 테스트는 충분히 넓고 장애물이 없는 공간에서 수행합니다.
* 전신 제어라고 하더라도 모바일 베이스는 Cartesian solver가 아닌 별도의 SE(2) velocity command로 실행됩니다.

