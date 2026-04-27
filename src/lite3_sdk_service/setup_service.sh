#!/bin/bash
# =============================================================================
# setup_service.sh
# 在机器狗本地运行，完成以下任务：
#   1. 编译 drdds / lite3_transfer / lite3_sdk_service（--packages-up-to）
#   2. 配置 lite3_sdk_service 的 systemd 开机自启动
#
# 前提：已将 src/drdds、src/lite3_transfer、src/lite3_sdk_service
#       通过 SCP 传输到机器狗，并在机器狗上运行本脚本。
#
# 使用方法:
#   chmod +x setup_service.sh
#   ./setup_service.sh [选项]
#
# 选项:
#   -r <distro>   ROS2 发行版 (默认: foxy)
#   -d <domain>   ROS_DOMAIN_ID (默认: 0)
#   -h            显示帮助信息
# =============================================================================

set -euo pipefail

# ===== 默认配置 =====
ROS_DISTRO="foxy"
ROS_DOMAIN_ID="0"
SERVICE_NAME="lite3_sdk_service"

# ===== 颜色输出 =====
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }
log_step()  { echo -e "\n${BOLD}${BLUE}==> $*${NC}"; }
log_done()  { echo -e "${GREEN}[DONE]${NC}  $*"; }

show_help() {
    cat <<EOF
使用方法: $(basename "$0") [选项]

在机器狗本地完成编译与 systemd 开机自启动配置。

选项:
  -r <distro>   ROS2 发行版     (默认: ${ROS_DISTRO})
  -d <domain>   ROS_DOMAIN_ID  (默认: ${ROS_DOMAIN_ID})
  -h            显示此帮助信息

示例:
  ./setup_service.sh
  ./setup_service.sh -r foxy -d 0
EOF
}

# ===== 解析参数 =====
while getopts "r:d:h" opt; do
    case $opt in
        r) ROS_DISTRO="$OPTARG" ;;
        d) ROS_DOMAIN_ID="$OPTARG" ;;
        h) show_help; exit 0 ;;
        *) log_error "未知选项: -$OPTARG"; show_help; exit 1 ;;
    esac
done

# ===== 路径推导 =====
# 脚本位于 <workspace>/src/lite3_sdk_service/setup_service.sh
# 工作空间根目录为脚本所在目录上两级
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"   # colcon workspace 根目录
INSTALL_DIR="${WS_DIR}/install"
EXEC_PATH="${INSTALL_DIR}/${SERVICE_NAME}/lib/${SERVICE_NAME}/sdk_service"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
CURRENT_USER="$(whoami)"

# ===== 打印配置摘要 =====
print_summary() {
    cat <<EOF

${BOLD}====================================================
  Lite3 SDK Service 本地部署脚本
====================================================${NC}
  工作空间:      ${WS_DIR}
  ROS2 发行版:   ${ROS_DISTRO}
  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}
  运行用户:      ${CURRENT_USER}
  服务名称:      ${SERVICE_NAME}
  可执行路径:    ${EXEC_PATH}
${BOLD}====================================================${NC}
EOF
}

# ===== 步骤 1: 环境检查 =====
step1_check_env() {
    log_step "步骤 1/3: 环境检查"

    # 检查 ROS2 setup 文件
    if [ ! -f "${ROS_SETUP}" ]; then
        log_error "未找到 ROS2 setup 文件: ${ROS_SETUP}"
        log_error "请确认 ROS2 ${ROS_DISTRO} 已正确安装，或通过 -r 指定正确的发行版。"
        exit 1
    fi
    log_info "ROS2 环境: ${ROS_SETUP}"

    # 检查 colcon
    if ! command -v colcon &>/dev/null; then
        log_error "未找到 colcon，请先安装: sudo apt-get install python3-colcon-common-extensions"
        exit 1
    fi
    log_info "colcon: $(command -v colcon)"

    # 检查源码包
    for pkg in drdds lite3_transfer lite3_sdk_service; do
        local pkg_path="${WS_DIR}/src/${pkg}"
        if [ ! -d "${pkg_path}" ]; then
            log_error "源码包不存在: ${pkg_path}"
            log_error "请先将对应包通过 SCP 传输到机器狗后再运行本脚本。"
            exit 1
        fi
        log_info "源码包 [OK]: ${pkg_path}"
    done

    log_done "环境检查通过。"
}

# ===== 步骤 2: 编译 =====
step2_build() {
    log_step "步骤 2/3: 编译（colcon build --packages-up-to ${SERVICE_NAME}）"

    cd "${WS_DIR}"
    source "${ROS_SETUP}"

    log_info "清理旧的构建产物（build / install / log）..."
    rm -rf build install log

    log_info "开始编译，请稍候..."
    echo "--------------------------------------------------------------"

    colcon build \
        --packages-up-to "${SERVICE_NAME}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    echo "--------------------------------------------------------------"

    # 验证可执行文件
    if [ ! -f "${EXEC_PATH}" ]; then
        log_error "编译后未找到可执行文件: ${EXEC_PATH}"
        log_error "请检查上方的编译日志。"
        exit 1
    fi

    log_done "编译成功，可执行文件: ${EXEC_PATH}"
}

# ===== 步骤 3: 配置 systemd 开机自启 =====
step3_setup_systemd() {
    log_step "步骤 3/3: 配置 systemd 开机自启动"

    local service_file="/etc/systemd/system/${SERVICE_NAME}.service"

    log_info "生成 service 文件: ${service_file}"

    sudo tee "${service_file}" > /dev/null <<SERVICEEOF
[Unit]
Description=Lite3 SDK Control UDP Service
After=network.target

[Service]
Type=simple
User=${CURRENT_USER}
Environment=ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
ExecStart=/bin/bash -lc 'source ${ROS_SETUP} && \
                         source ${INSTALL_DIR}/setup.bash && \
                         exec ${EXEC_PATH}'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
SERVICEEOF

    log_info "重新加载 systemd 守护进程..."
    sudo systemctl daemon-reload

    log_info "启用开机自启..."
    sudo systemctl enable "${SERVICE_NAME}"

    log_info "启动服务..."
    sudo systemctl restart "${SERVICE_NAME}"

    # 等待服务稳定后检查状态
    sleep 2
    if systemctl is-active --quiet "${SERVICE_NAME}"; then
        log_done "服务运行中 (active)。"
    else
        log_warn "服务可能未正常启动，请检查日志:"
        log_warn "  sudo journalctl -u ${SERVICE_NAME} -n 30"
    fi
}

# ===== 打印完成信息 =====
print_finish() {
    cat <<EOF

${BOLD}${GREEN}====================================================
  部署完成！
====================================================${NC}
服务管理命令:
  查看状态:  sudo systemctl status ${SERVICE_NAME}
  查看日志:  sudo journalctl -u ${SERVICE_NAME} -f
  重启服务:  sudo systemctl restart ${SERVICE_NAME}
  停止服务:  sudo systemctl stop ${SERVICE_NAME}
  禁用自启:  sudo systemctl disable ${SERVICE_NAME}

UDP 控制命令（端口 12122）:
  启动 transfer: echo "on"  | nc -u 127.0.0.1 12122
  停止 transfer: echo "off" | nc -u 127.0.0.1 12122
  设置发布频率:  echo "50"  | nc -u 127.0.0.1 12122
${BOLD}${GREEN}====================================================${NC}
EOF
}

# ===== 主流程 =====
main() {
    print_summary
    step1_check_env
    step2_build
    step3_setup_systemd
    print_finish
}

main "$@"
