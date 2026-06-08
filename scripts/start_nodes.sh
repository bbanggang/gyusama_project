#!/usr/bin/env bash
# start_nodes.sh — RPi5 전체 노드 기동 (예외 처리 강화 버전)
#
# 사용법:
#   ./scripts/start_nodes.sh                   # 전체 (차선+장애물+behavior+제어)
#   ./scripts/start_nodes.sh --debug           # PUBLISH_DEBUG=1 (rqt 시각 확인)
#   ./scripts/start_nodes.sh --lane-only       # 차선 추종만 (장애물/behavior 제외)
#                                              # DIRECT_CMD_VEL=1 — inference → /cmd_vel 직접
#   ./scripts/start_nodes.sh --no-obstacle     # 장애물 노드만 제외 (behavior+inference+control)
#
# 옵션은 조합 가능: ./scripts/start_nodes.sh --lane-only --debug
#
# 주행 시작:
#   sudo docker attach inference-node          # SSH 들어가서
#                                              # 's' 키 입력 (수동 시작 — 정책)
#
# 환경변수:
#   RPI5_HOST=192.168.0.155 (디폴트)
#   RPI5_USER=rapi5
#   RPI5_DIR=~/gyusama-project
#
# 정책: 자동 주행 (AUTOSTART) 없음. 's' 키 누르기 전까지 모터 정지 유지.

set -euo pipefail

RPI5_HOST="${RPI5_HOST:-192.168.0.155}"
RPI5_USER="${RPI5_USER:-rapi5}"
RPI5_DIR="${RPI5_DIR:-~/gyusama-project}"

# 색상
R="\e[31m"; G="\e[32m"; Y="\e[33m"; C="\e[36m"; N="\e[0m"
ok()   { echo -e "  ${G}✓${N} $1"; }
warn() { echo -e "  ${Y}⚠${N} $1"; }
fail() { echo -e "  ${R}✗${N} $1"; }

# ── 옵션 파싱 ───────────────────────────────────────────────────────────
PUBLISH_DEBUG=0
LANE_ONLY=0
NO_OBSTACLE=0
for arg in "$@"; do
    case $arg in
        --debug)        PUBLISH_DEBUG=1 ;;
        --lane-only)    LANE_ONLY=1 ;;
        --no-obstacle)  NO_OBSTACLE=1 ;;
        --autostart)
            echo -e "${R}[ERROR]${N} --autostart 옵션은 폐기됨 (안전 정책)."
            echo "  주행 시작은 항상 'docker attach inference-node' → 's' 키로만 가능합니다."
            exit 1 ;;
        -h|--help)
            sed -n '2,22p' "$0"
            exit 0 ;;
    esac
done

# ── 노드 조합 결정 ─────────────────────────────────────────────────────
# 디폴트(전체):       inference + control + behavior + obstacle
# --lane-only:        inference + control 만, DIRECT_CMD_VEL=1 (inference → /cmd_vel 직접)
# --no-obstacle:      inference + control + behavior (obstacle만 제외)
SERVICES=""        # 빈 문자열 = compose default (전체 기동)
DIRECT_CMD_VEL=0
MODE_LABEL="전체 (차선+장애물+behavior+제어)"
EXPECTED_CONTAINERS=(inference-node control-node behavior-node obstacle-node)

if [[ $LANE_ONLY -eq 1 ]]; then
    SERVICES="inference-node control-node"
    DIRECT_CMD_VEL=1
    MODE_LABEL="차선 추종 단독 (inference → /cmd_vel 직접, behavior/obstacle 제외)"
    EXPECTED_CONTAINERS=(inference-node control-node)
elif [[ $NO_OBSTACLE -eq 1 ]]; then
    SERVICES="inference-node control-node behavior-node"
    DIRECT_CMD_VEL=0
    MODE_LABEL="장애물 노드만 제외 (behavior 합성은 활성)"
    EXPECTED_CONTAINERS=(inference-node control-node behavior-node)
fi

echo -e "${C}=== RPi5 노드 기동 ===${N}"
echo "  대상:           ${RPI5_USER}@${RPI5_HOST}"
echo "  모드:           ${MODE_LABEL}"
echo "  시작 방법:      수동 ('s' 키 필요)"
echo "  PUBLISH_DEBUG=${PUBLISH_DEBUG}"
echo "  DIRECT_CMD_VEL=${DIRECT_CMD_VEL}"
echo ""

# SSH 한 번에 묶기 위한 헬퍼
remote() {
    ssh -o ConnectTimeout=8 -i ~/.ssh/id_ed25519 "${RPI5_USER}@${RPI5_HOST}" "$@"
}

# ─────────────────────────────────────────────────────────────────────
# [0/5] 사전 점검 — RPi5 도달성 + sudo 사용 가능
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}[0/5] 사전 점검 — RPi5 연결 및 권한${N}"
if ! ping -c 1 -W 3 "${RPI5_HOST}" >/dev/null 2>&1; then
    fail "RPi5 (${RPI5_HOST}) 도달 불가 — 전원 또는 네트워크 확인"
    exit 1
fi
ok "ping 응답"

if ! remote 'echo ok' >/dev/null 2>&1; then
    fail "SSH 연결 실패 — 키 또는 사용자 확인"
    exit 1
fi
ok "SSH 연결"
echo ""

# ─────────────────────────────────────────────────────────────────────
# [1/5] NPU 인식 확인 + 자동 복구
#   증상: 커널 자동 업데이트 후 DKMS 모듈이 새 커널용으로 빌드 안 됨
#         → /dev/hailo0 없음 → inference-node가 HEF 로드 못 함
#   복구: dkms autoinstall + modprobe hailo_pci
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}[1/5] NPU 인식 확인${N}"
NPU_STATUS=$(remote 'bash -c "
    if [ -e /dev/hailo0 ]; then echo OK
    else
        # PCIe 인식 확인 — 없으면 HW 미장착
        if ! lspci 2>/dev/null | grep -qi hailo; then echo NO_PCIE
        elif ! lsmod | grep -q hailo_pci; then echo MODULE_NOT_LOADED
        else echo DEVICE_MISSING
        fi
    fi
"')

case "$NPU_STATUS" in
    OK)
        ok "/dev/hailo0 정상" ;;
    NO_PCIE)
        fail "Hailo PCIe 인식 안 됨 (HW 점검 필요)"
        exit 1 ;;
    MODULE_NOT_LOADED|DEVICE_MISSING)
        warn "hailo_pci 모듈 미로드 — 자동 복구 시도 (커널 업데이트 가능성)"
        echo "    rapi5 비밀번호 필요 (10초 안에)"
        remote 'bash -c "
            CUR_KERN=$(uname -r)
            # 현재 커널 헤더 없으면 설치
            if ! dpkg -l linux-headers-\$CUR_KERN 2>/dev/null | grep -q ^ii; then
                echo \"  + 새 커널 헤더 설치: linux-headers-\$CUR_KERN\"
                echo rapi5 | sudo -S apt-get install -y linux-headers-\$CUR_KERN >/dev/null 2>&1
            fi
            # DKMS 재빌드
            echo \"  + DKMS autoinstall\"
            echo rapi5 | sudo -S dkms autoinstall 2>&1 | tail -3
            # 모듈 로드
            echo rapi5 | sudo -S modprobe hailo_pci 2>&1
            sleep 2
            if [ -e /dev/hailo0 ]; then echo \"  ✓ /dev/hailo0 복구 성공\"
            else echo \"  ✗ 복구 실패\"; exit 1
            fi
        "' || { fail "NPU 복구 실패 — 수동 진단 필요"; exit 1; }
        ok "NPU 자동 복구 완료" ;;
esac
echo ""

# ─────────────────────────────────────────────────────────────────────
# [2/5] Docker 컨테이너 기동
#   - PUBLISH_DEBUG 환경변수 전달
#   - AUTOSTART 는 명시적으로 0 으로 고정 (정책)
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}[2/5] Docker 컨테이너 기동${N}"
# lane-only 등 부분 기동 시 기존 다른 노드는 명시적 stop (메모리/잔존 명령 방지)
if [[ -n "$SERVICES" ]]; then
    remote "cd ${RPI5_DIR}/docker && docker compose stop behavior-node obstacle-node 2>&1 | tail -3" || true
fi
# 환경변수 + 선택적 서비스 기동
remote "cd ${RPI5_DIR}/docker && \
        PUBLISH_DEBUG=${PUBLISH_DEBUG} \
        AUTOSTART=0 \
        DIRECT_CMD_VEL=${DIRECT_CMD_VEL} \
        docker compose up -d ${SERVICES} 2>&1 | tail -6"
sleep 5

# 기대 컨테이너만 검증 (lane-only 면 2개만)
RUNNING=$(remote "docker ps --format '{{.Names}}' | grep -E 'inference|control|behavior|obstacle' || true")
for c in "${EXPECTED_CONTAINERS[@]}"; do
    if echo "$RUNNING" | grep -q "^${c}\$"; then
        ok "${c} 기동"
    else
        fail "${c} 기동 실패"
        echo "    로그: ssh ${RPI5_USER}@${RPI5_HOST} 'docker logs ${c}'"
        exit 1
    fi
done
# 모드에 따라 의도적으로 제외된 노드도 안내
if [[ $LANE_ONLY -eq 1 ]]; then
    echo "  ✕ behavior-node / obstacle-node — 의도적 제외 (lane-only)"
elif [[ $NO_OBSTACLE -eq 1 ]]; then
    echo "  ✕ obstacle-node — 의도적 제외 (--no-obstacle)"
fi
echo ""

# ─────────────────────────────────────────────────────────────────────
# [3/5] camera_node 기동
#   - 기존 좀비 프로세스 정리 후 새로 시작
#   - 환경변수 (ROS_DOMAIN_ID, FASTDDS) 명시
#   - 발행 검증
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}[3/5] camera_node 기동${N}"
# 패턴 "camera_ros/camera_node" — 실제 실행 binary path 매칭, ssh 명령 자체와 충돌 X
remote 'bash -c "
    # 기존 프로세스 강제 정리 (좀비/lock holder 제거)
    PIDS=\$(pgrep -x camera_node 2>/dev/null)
    if [ -n \"\$PIDS\" ]; then
        echo \"  + 기존 camera_node 종료 (PIDS: \$PIDS)\"
        echo \"\$PIDS\" | xargs kill -9 2>/dev/null
        sleep 3
    fi
    # 새로 시작 (환경변수 명시)
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=1
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    nohup ros2 run camera_ros camera_node --ros-args \
        -p FrameDurationLimits:=\"[33333,33333]\" \
        > /tmp/camera_node.log 2>&1 &
    echo \$! > /tmp/camera_node.pid
    sleep 6
    NEWPID=\$(pgrep -x camera_node)
    if [ -n \"\$NEWPID\" ]; then
        echo \"  + camera_node PID: \$NEWPID\"
    else
        echo \"  ✗ camera_node 시작 실패 — /tmp/camera_node.log 확인\"
        exit 1
    fi
"' || { fail "camera_node 기동 실패"; exit 1; }
ok "camera_node 시작"

# 발행 검증 (5초 동안 hz 측정)
echo "  발행 검증 중 (5초)..."
HZ=$(remote 'bash -c "
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=1
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    timeout 5 ros2 topic hz /camera/image_raw 2>&1 | grep -oE \"average rate: [0-9.]+\" | head -1
"' || true)
if [[ -n "$HZ" ]]; then
    ok "/camera/image_raw 발행 — ${HZ}"
else
    warn "/camera/image_raw 발행 미감지 — 카메라 좀비 가능성"
    echo "    로그: ssh ${RPI5_USER}@${RPI5_HOST} 'tail -20 /tmp/camera_node.log'"
fi
echo ""

# ─────────────────────────────────────────────────────────────────────
# [4/5] Dynamixel 모터 토크 활성화
#   증상: turtlebot3_node 가 한 번 죽었다 살아나면 토크 자동 비활성화
#   해결: motor_power 서비스 호출
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}[4/5] Dynamixel 모터 토크 활성화${N}"
TORQUE=$(remote 'bash -c "
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=1
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    # 서비스 가용까지 대기
    for i in {1..10}; do
        if ros2 service list 2>/dev/null | grep -q /motor_power; then break; fi
        sleep 1
    done
    timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool \"{data: true}\" 2>&1 \
        | grep -oE \"success=(True|False)\" | head -1
"' || true)
if [[ "$TORQUE" == "success=True" ]]; then
    ok "모터 토크 활성화"
else
    warn "모터 토크 활성화 실패 — turtlebot3_node 또는 USB 점검"
    echo "    수동 시도:"
    echo "      ssh ${RPI5_USER}@${RPI5_HOST}"
    echo "      ros2 service call /motor_power std_srvs/srv/SetBool '{data: true}'"
fi
echo ""

# ─────────────────────────────────────────────────────────────────────
# [5/5] inference-node HEF 로드 확인
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}[5/5] inference-node HEF 로드 검증${N}"
HEF_LOG=$(remote "docker logs inference-node 2>&1 | grep -E 'HEF 로드|HEF 모델 사용|준비 완료' | tail -3" || true)
if echo "$HEF_LOG" | grep -q "HEF 로드"; then
    ok "HEF 정상 로드"
    echo "$HEF_LOG" | sed 's/^/    /'
else
    warn "HEF 로드 로그 없음 — 로그 직접 확인 필요"
    echo "    ssh ${RPI5_USER}@${RPI5_HOST} 'docker logs inference-node --tail 20'"
fi
echo ""

# ─────────────────────────────────────────────────────────────────────
# 완료 + 사용법 안내
# ─────────────────────────────────────────────────────────────────────
echo -e "${C}=== 기동 완료 ===${N}"
echo "  모드: ${MODE_LABEL}"
if [[ $LANE_ONLY -eq 1 ]]; then
    echo "  → 추론 결과가 /cmd_vel 로 직접 발행됩니다 (장애물 회피/blending 없음)"
fi
echo ""
echo "  주행 시작 (수동 — 정책):"
echo -e "    ${G}ssh -t ${RPI5_USER}@${RPI5_HOST} docker attach inference-node${N}"
echo "    → 's' 키 = 시작"
echo "    → 'q' 키 = 정지"
echo "    → Ctrl+P, Ctrl+Q (순차) = detach (안전)"
echo "    → Ctrl+C 금지 (컨테이너 죽음)"
echo ""
echo "  rqt 시각 확인 (PC, 별도 터미널):"
echo "    FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_peer.xml ROS_DOMAIN_ID=1 \\"
echo "      ros2 run rqt_image_view rqt_image_view"
echo ""
echo "  전체 종료:"
echo "    ./scripts/stop_nodes.sh"
