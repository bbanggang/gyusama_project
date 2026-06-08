#!/usr/bin/env bash
# stop_nodes.sh — RPi5 전체 노드 안전 종료 (예외 처리 강화 버전)
#
# 종료 순서 (안전 우선):
#   1. cmd_vel 0,0 발행 — 모터 즉시 정지
#   2. inference-node 종료 — 새 명령 발행 차단
#   3. camera_node 종료 — 카메라 디바이스 해제
#   4. 나머지 컨테이너 종료
#   5. (선택) 모터 토크 비활성화
#
# 사용법:
#   ./scripts/stop_nodes.sh             # 기본 (안전 종료)
#   ./scripts/stop_nodes.sh --force     # 강제 종료 (cmd_vel 정지 단계 스킵)
#   ./scripts/stop_nodes.sh --torque-off # 종료 후 모터 토크도 끔

set -euo pipefail

RPI5_HOST="${RPI5_HOST:-192.168.0.155}"
RPI5_USER="${RPI5_USER:-rapi5}"
RPI5_DIR="${RPI5_DIR:-~/gyusama-project}"

R="\e[31m"; G="\e[32m"; Y="\e[33m"; C="\e[36m"; N="\e[0m"
ok()   { echo -e "  ${G}✓${N} $1"; }
warn() { echo -e "  ${Y}⚠${N} $1"; }
fail() { echo -e "  ${R}✗${N} $1"; }

FORCE=0
TORQUE_OFF=0
for arg in "$@"; do
    case $arg in
        --force)      FORCE=1 ;;
        --torque-off) TORQUE_OFF=1 ;;
        -h|--help)
            sed -n '2,13p' "$0"; exit 0 ;;
    esac
done

remote() {
    ssh -o ConnectTimeout=8 -i ~/.ssh/id_ed25519 "${RPI5_USER}@${RPI5_HOST}" "$@"
}

echo -e "${C}=== RPi5 노드 안전 종료 ===${N}"
echo "  대상: ${RPI5_USER}@${RPI5_HOST}"
echo ""

# RPi5 도달성 (이미 꺼져 있어도 OK 처리)
if ! ping -c 1 -W 3 "${RPI5_HOST}" >/dev/null 2>&1; then
    warn "RPi5 도달 불가 — 이미 종료된 상태일 수 있음"
    exit 0
fi

# ─────────────────────────────────────────────────────────────────────
# [1/4] 모터 즉시 정지 (cmd_vel 0,0 1초간 강제 발행)
#   inference-node 가 자동 발행하는 cmd_vel 과 경합하지 않도록
#   inference-node 정지 직전에 0,0 발행
# ─────────────────────────────────────────────────────────────────────
if [[ $FORCE -eq 0 ]]; then
    echo -e "${C}[1/4] 모터 안전 정지 (cmd_vel 0,0 발행)${N}"
    remote 'bash -c "
        source /opt/ros/jazzy/setup.bash
        export ROS_DOMAIN_ID=1
        export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
        # 1초 동안 0,0 발행 (10Hz × 10회)
        timeout 1.5 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
            \"{header: {frame_id: stop}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}\" \
            >/dev/null 2>&1 || true
    "' && ok "0,0 발행 완료" || warn "발행 실패 (이미 토픽 없음)"
    echo ""
else
    warn "--force 모드 — 모터 정지 단계 스킵"
    echo ""
fi

# ─────────────────────────────────────────────────────────────────────
# [2/4] inference-node 우선 종료 (cmd_vel 추가 발행 차단)
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}[2/4] inference-node 종료${N}"
INF_RUNNING=$(remote "docker ps --filter name=inference-node --format '{{.Names}}'" || true)
if [[ -n "$INF_RUNNING" ]]; then
    remote "cd ${RPI5_DIR}/docker && docker compose stop inference-node 2>&1 | tail -2"
    ok "inference-node 정지"
else
    warn "inference-node 실행 중 아님"
fi
echo ""

# ─────────────────────────────────────────────────────────────────────
# [3/4] camera_node 종료 (카메라 디바이스 해제)
#   PID 파일 우선, 없으면 pgrep
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}[3/4] camera_node 종료${N}"
# 패턴 "camera_ros/camera_node" — 실제 binary, ssh 명령 자체와 충돌 X
remote 'bash -c "
    KILLED=0
    # PID 파일이 있으면 우선 사용
    if [ -f /tmp/camera_node.pid ]; then
        PID=\$(cat /tmp/camera_node.pid 2>/dev/null)
        if [ -n \"\$PID\" ] && kill -0 \$PID 2>/dev/null; then
            kill -TERM \$PID 2>/dev/null
            sleep 2
            kill -KILL \$PID 2>/dev/null || true
            KILLED=1
        fi
        rm -f /tmp/camera_node.pid
    fi
    # 추가로 pgrep 으로 잔존 프로세스 정리 (좀비 방지)
    PIDS=\$(pgrep -x camera_node 2>/dev/null || true)
    if [ -n \"\$PIDS\" ]; then
        echo \"\$PIDS\" | xargs kill -9 2>/dev/null || true
        KILLED=1
    fi
    if [ \$KILLED -eq 1 ]; then echo OK; else echo NOT_RUNNING; fi
"' | grep -q OK && ok "camera_node 종료" || warn "camera_node 실행 중 아님"
echo ""

# ─────────────────────────────────────────────────────────────────────
# [4/4] 나머지 컨테이너 종료
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}[4/4] 컨테이너 종료 (control/behavior/obstacle)${N}"
remote "cd ${RPI5_DIR}/docker && docker compose down 2>&1 | tail -5"
ok "컨테이너 모두 정리"
echo ""

# ─────────────────────────────────────────────────────────────────────
# (선택) 모터 토크 끄기
# ─────────────────────────────────────────────────────────────────────
if [[ $TORQUE_OFF -eq 1 ]]; then
    warn "모터 토크 비활성화는 control-node 종료 후 무의미 (이미 통신 두절)"
    echo "    → 다음 부팅 시 start_nodes.sh 가 자동으로 토크 활성화"
fi

# ─────────────────────────────────────────────────────────────────────
# 최종 검증
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}=== 정리 상태 검증 ===${N}"
LEFT_CONT=$(remote "docker ps --format '{{.Names}}' | grep -E 'inference|control|behavior|obstacle' || true")
if [[ -z "$LEFT_CONT" ]]; then
    ok "프로젝트 컨테이너 모두 종료됨"
else
    warn "남아있는 컨테이너:"
    echo "$LEFT_CONT" | sed 's/^/    /'
fi

LEFT_CAM=$(remote "pgrep -xa camera_node 2>/dev/null || true")
if [[ -z "$LEFT_CAM" ]]; then
    ok "camera_node 잔존 프로세스 없음"
else
    warn "camera_node 잔존:"
    echo "$LEFT_CAM" | sed 's/^/    /'
fi

echo ""
echo -e "${C}=== 종료 완료 ===${N}"
