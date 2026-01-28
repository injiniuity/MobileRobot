# nav2_waypoint
키보드 입력을 GUI/매니퓰레이터 신호로 분리하고, 시나리오 상태머신으로 Nav2 목표를 보내는 ROS2 Python

<br>
<br>

## 개발 환경
- OS: Ubuntu 22.04 LTS
- ROS 2: Jazzy
- 언어: Python (rclpy)
- 네비게이션: Nav2
  
<br>
<br>

## 노드 설명
<img width="1257" height="343" alt="image" src="https://github.com/user-attachments/assets/9bae6b09-2877-4784-8274-0b1161acfb76" />

| 파일 | 노드 | 설명 | 
|------|------|------|
| keyboard_trigger.py | /keyboard_trigger | 키보드 입력 처리 |
| gui_trigger.py | /gui_trigger | 임의적으로 만든 gui 역할을 대신하는 노드<br>• P : 대기 장소 이동<br>• 1 / 3 / 4 : 업무 할당 (부품입고 / 모듈입고 / 모듈출고)<br>• SPACE : 상차 완료<br>• U : 하차 완료 |
| manip_trigger.py | /manip_trigger | 임의적으로 만든 매니퓰레이터 역할을 대신하는 노드<br>• Q : 하차 완료 (부품입고)<br>• W : 상차 완료 (모듈입고) |
| Nav2_classes.py | /go_to_goal | Nav2를 통해 로봇을 목표 지점으로 이동시키고 로그 관리하는 노드 |
| scenario_manager.py | /scenario_manager | 중앙 시나리오 제어 노드<br>• /gui_cmd, /manip_cmd 입력 처리<br>• 현재 업무에 맞는 입력만 허용<br>• 이동 포인트 결정<br>• go_to_goal 액션으로 목표 전송<br>• 도착 시 /arrived_point 발행<br>• 업무 변경(1/3/4), 대기(P) 같은 예외 입력 처리<br>• 키 입력 → 상태 전이 → 이동 명령 담당 |

<br>
<br>

## 구성 파일
nav2_waypoint/
- package.xml
- setup.py
- setup.cfg
- Nav2_prams.yaml : /home/pinky/pinky_pro/src/pinky_pro/pinky_navigation/params/nav2_params.yaml Nav2 파라미터 파일
- goal_points.json : 목표 포인트 좌표
- run_scenario.py : 전체 노드를 한 번에 실행하는 런처
- scenario_manager.py : 시나리오 FSM (GUI/매니퓰레이터 입력 → 목표 전환)
- Nav2_classes.py : `go_to_goal` 액션 서버/브리지
- keyboard_trigger.py : 키보드 입력 → `/keyboard_raw`
- gui_trigger.py : `/keyboard_raw` → `/gui_cmd` 필터
- manip_trigger.py : `/keyboard_raw` → `/manip_cmd` 필터
  
<br>
<br>

## 주요 토픽/액션
### 토픽(Topic)
- `/keyboard_raw` (String): 키보드 원시 입력
- `/gui_cmd` (String): GUI 명령
- `/manip_cmd` (String): 매니퓰레이터 명령
- `/arrived_point` (String): 도착 포인트 알림
  
<br>
<br>

### 액션(Action)
- `/go_to_goal` (NavigateToPose action): 시나리오 → 네비게이션 브리지
  
<br>
<br>

## 키 매핑
### GUI 입력(`/gui_cmd`)
- `1` : 업무1 시작 → point1
- `3` : 업무2 시작 → point3
- `4` : 업무3 시작 → point4
- `P` : 대기 장소(point0)
- `SPACE` : GUI 상차 완료 (업무1/3에서 사용)
- `U` : GUI 하차 완료 (업무2/3에서 사용)
  
<br>
<br>

### 매니퓰레이터 입력(`/manip_cmd`)
- `Q` : 하차 완료 (업무1에서 사용)
- `W` : 상차 완료 (업무2에서 사용)
  
<br>
<br>

> 입력의 유효성은 `scenario_manager`의 상태에 따라 결정됩니다.  
> 예: `WAIT_SPACE_FROM_3` 상태에서는 `W`만 유효합니다.

<br>
<br>

## 패키지 실행 방법
```bash
[Robot]
cd /home/pinky/pinky_pro
colcon build --packages-select nav2_waypoint --symlink-install
source /home/pinky/pinky_pro/install/setup.bash
ros2 run nav2_waypoint run_scenario
ros2 launch pinky_bringup bringup_robot.launch.xml
ros2 launch pinky_navigation bringup_launch.xml map:=jini_EmptyMap.yaml

[PC]
ros2 launch pinky_navigation map_view.launch.xml
```

<br>
<br>

## 동작 흐름(요약)
- 키보드 입력 → `/keyboard_raw`
- GUI/매니퓰레이터 필터 → `/gui_cmd`, `/manip_cmd`
- `scenario_manager`가 상태머신으로 목표 설정
- `go_to_goal`이 Nav2로 목표 전달
- 도착 시 `/arrived_point` 발행



