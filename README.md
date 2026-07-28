# LeRobot Teleoperator Box

Meta Quest VR을 사용하여 RB-Y1 양팔의 Cartesian End-Effector pose를 제어하기 위한 LeRobot Teleoperator 패키지입니다.

## Installation

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

설치 확인:

```bash
python -m pip show lerobot-teleoperator-box
```

Import 확인:

```bash
python -c "from lerobot_teleoperator_box import BoxVr, BoxVrConfig; print('IMPORT OK')"
```

## Teleoperate

Meta Quest VR로 RBY1을 직접 조작합니다.

Meta Quest 앱 실행 후 양쪽 컨트롤러의 아무 버튼이나 한 번 눌러 컨트롤러 pose 스트리밍을 활성화합니다.


```
lerobot-teleoperate \
  --robot.type=rby1 \
  --robot.address=192.168.30.1:50051 \
  --robot.use_right_arm=true \
  --robot.use_left_arm=true \
  --robot.use_gripper=false \
  --robot.action_mode=ee \
  --teleop.type=box_vr \
  --teleop.local_ip=192.168.0.132 \
  --teleop.local_port=5005 \
  --teleop.meta_quest_ip=192.168.0.69 \
  --teleop.meta_quest_port=6000 \
  --teleop.send_handshake=true \
  --teleop.use_right_arm=true \
  --teleop.use_left_arm=true \
  --teleop.use_gripper=false 
```


## Record

PC와 Meta Quest가 동일한 네트워크에 연결되어 있어야 합니다.

* `teleop.local_ip`: LeRobot을 실행하는 PC의 IP
* `teleop.meta_quest_ip`: Meta Quest의 IP
* `teleop.local_port`: Quest pose 수신 포트
* `teleop.meta_quest_port`: Quest handshake 포트

**예시 :**

```bash
lerobot-record \
  --robot.type=rby1 \
  --robot.address=192.168.30.1:50051 \
  --robot.use_right_arm=true \
  --robot.use_left_arm=true \
  --robot.use_gripper=false \
  --robot.action_mode=ee \
  --teleop.type=box_vr \
  --teleop.local_ip=192.168.0.132 \
  --teleop.local_port=5005 \
  --teleop.meta_quest_ip=192.168.0.69 \
  --teleop.meta_quest_port=6000 \
  --teleop.send_handshake=true \
  --teleop.use_right_arm=true \
  --teleop.use_left_arm=true \
  --teleop.use_gripper=false \
  --dataset.repo_id=<hf_username>/<dataset_name> \
  --dataset.single_task="Teleoperate both arms of RB-Y1 using Meta Quest VR." \
  --dataset.num_episodes=50 \
  --dataset.fps=30
```


## Controls

* Right grip: 오른팔 Cartesian pose 추종
* Left grip: 왼팔 Cartesian pose 추종
* Grip 해제: 마지막 EE target 유지
* Trigger: gripper control 예정

## Notes

`teleop.local_ip`에는 다음과 같이 실제 네트워크 인터페이스의 IP를 입력해야 합니다.

```bash
hostname -I
```

