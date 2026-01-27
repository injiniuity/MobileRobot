# nav2_waypoint

키보드 입력을 GUI/매니퓰레이터 신호로 분리하고, 시나리오 상태머신으로 Nav2 목표를 보내는 ROS2 Python

## 개발 환경
- OS: Ubuntu 22.04 LTS
- ROS 2: Jazzy
- 언어: Python (rclpy)
- 네비게이션: Nav2

## 구성 파일
nav2_waypoint/
- package.xml
- setup.py
- setup.cfg
- README.md
- goal_points.json : 목표 포인트 좌표
- run_scenario.py : 전체 노드를 한 번에 실행하는 런처
- scenario_manager.py : 시나리오 FSM (GUI/매니퓰레이터 입력 → 목표 전환)
- Nav2_classes.py : `go_to_goal` 액션 서버/브리지
- keyboard_trigger.py : 키보드 입력 → `/keyboard_raw`
- gui_trigger.py : `/keyboard_raw` → `/gui_cmd` 필터
- manip_trigger.py : `/keyboard_raw` → `/manip_cmd` 필터

## 주요 토픽/액션
### 토픽(Topic)
- `/keyboard_raw` (String): 키보드 원시 입력
- `/gui_cmd` (String): GUI 명령
- `/manip_cmd` (String): 매니퓰레이터 명령
- `/arrived_point` (String): 도착 포인트 알림

### 액션(Action)
- `/go_to_goal` (NavigateToPose action): 시나리오 → 네비게이션 브리지

## 키 매핑
### GUI 입력(`/gui_cmd`)
- `1` : 업무1 시작 → point1
- `3` : 업무2 시작 → point3
- `4` : 업무3 시작 → point4
- `P` : 대기 장소(point0)
- `SPACE` : GUI 상차 완료 (업무1/3에서 사용)
- `U` : GUI 하차 완료 (업무2/3에서 사용)

### 매니퓰레이터 입력(`/manip_cmd`)
- `Q` : 하차 완료 (업무1에서 사용)
- `W` : 상차 완료 (업무2에서 사용)

> 입력의 유효성은 `scenario_manager`의 상태에 따라 결정됩니다.  
> 예: `WAIT_SPACE_FROM_3` 상태에서는 `W`만 유효합니다.

## 실행 방법
```bash
cd /home/pinky/pinky_pro
colcon build --packages-select nav2_waypoint --symlink-install
source /home/pinky/pinky_pro/install/setup.bash
ros2 run nav2_waypoint run_scenario
```

## 동작 흐름(요약)
- 키보드 입력 → `/keyboard_raw`
- GUI/매니퓰레이터 필터 → `/gui_cmd`, `/manip_cmd`
- `scenario_manager`가 상태머신으로 목표 설정
- `go_to_goal`이 Nav2로 목표 전달
- 도착 시 `/arrived_point` 발행



