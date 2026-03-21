# Architecture

## 동작 흐름
1. 외부 노드가 `/jetank_cmd` 토픽으로 명령 publish
2. 제어 노드가 명령 subscribe 후 서보/구동 제어
3. 현재 상태를 `/robot_state` 토픽으로 publish
4. 관측 노드에서 상태 echo로 확인
