#!/bin/bash
# =============================================================================
# setup_service.sh
# Run on the robot host to complete the following tasks:
#   1. Build drdds / lite3_transfer / lite3_sdk_service (--packages-up-to)
#   2. Configure systemd auto-start for lite3_sdk_service
#
# Prerequisite: src/drdds, src/lite3_transfer, and src/lite3_sdk_service
# must already be copied to the robot host via SCP, and this script must be
# executed on the robot host.
#
# Usage:
#   chmod +x setup_service.sh
#   ./setup_service.sh [options]
#
# Options:
#   -r <distro>   ROS2 distro (default: foxy)
#   -d <domain>   ROS_DOMAIN_ID (default: 0)
#   -h            Show help
# =============================================================================

set -euo pipefail

# ===== Default configuration =====
ROS_DISTRO="foxy"
ROS_DOMAIN_ID="0"
SERVICE_NAME="lite3_sdk_service"

# ===== Colored output =====
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

# Source a file with nounset disabled. ROS2 / colcon setup scripts reference
# unbound variables (e.g. AMENT_TRACE_SETUP_FILES), which would abort under
# `set -u`. Wrapping the source keeps the rest of the script strict.
source_env() {
    set +u
    # shellcheck disable=SC1090
    source "$1"
    set -u
}

show_help() {
    cat <<EOF
Usage: $(basename "$0") [options]

Build Lite3 SDK service on the robot host and configure systemd auto-start.

This script only creates and enables lite3_sdk_service.service.
lite3_transfer is not registered as a standalone systemd service.
It is started or stopped on demand by sdk_service after UDP control commands.

Options:
  -r <distro>   ROS2 distro     (default: ${ROS_DISTRO})
  -d <domain>   ROS_DOMAIN_ID   (default: ${ROS_DOMAIN_ID})
  -h            Show this help message

Examples:
  ./setup_service.sh
  ./setup_service.sh -r foxy -d 0
EOF
}

# ===== Parse arguments =====
while getopts "r:d:h" opt; do
    case $opt in
        r) ROS_DISTRO="$OPTARG" ;;
        d) ROS_DOMAIN_ID="$OPTARG" ;;
        h) show_help; exit 0 ;;
        *) log_error "Unknown option: -$OPTARG"; show_help; exit 1 ;;
    esac
done

# ===== Path resolution =====
# Script location: <workspace>/src/lite3_sdk_service/setup_service.sh
# Workspace root: two levels above the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"   # colcon workspace root
INSTALL_DIR="${WS_DIR}/install"
EXEC_PATH="${INSTALL_DIR}/${SERVICE_NAME}/lib/${SERVICE_NAME}/sdk_service"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
CURRENT_USER="$(whoami)"

# ===== Print configuration summary =====
print_summary() {
    cat <<EOF

${BOLD}====================================================
  Lite3 SDK Service Local Deployment Script
====================================================${NC}
  Workspace:      ${WS_DIR}
  ROS2 distro:    ${ROS_DISTRO}
  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}
  Run user:       ${CURRENT_USER}
  Service name:   ${SERVICE_NAME}
  Executable:     ${EXEC_PATH}
${BOLD}====================================================${NC}
EOF
}

# ===== Step 1: environment check =====
step1_check_env() {
    log_step "Step 1/3: Environment check"

    # Check ROS2 setup file
    if [ ! -f "${ROS_SETUP}" ]; then
        log_error "ROS2 setup file not found: ${ROS_SETUP}"
        log_error "Make sure ROS2 ${ROS_DISTRO} is installed, or pass the correct distro with -r."
        exit 1
    fi
    log_info "ROS2 environment: ${ROS_SETUP}"

    # Check colcon
    if ! command -v colcon &>/dev/null; then
        log_error "colcon not found. Install it first: sudo apt-get install python3-colcon-common-extensions"
        exit 1
    fi
    log_info "colcon: $(command -v colcon)"

    # Check source packages
    for pkg in drdds lite3_transfer lite3_sdk_service; do
        local pkg_path="${WS_DIR}/src/${pkg}"
        if [ ! -d "${pkg_path}" ]; then
            log_error "Source package not found: ${pkg_path}"
            log_error "Copy the required packages to the robot host via SCP before running this script."
            exit 1
        fi
        log_info "Source package [OK]: ${pkg_path}"
    done

    log_done "Environment check passed."
}

# ===== Detect foreign workspaces sourced in the environment =====
# Any path in AMENT_PREFIX_PATH that is neither the ROS2 install nor this
# workspace's install is a foreign overlay (e.g. an old workspace sourced from
# ~/.bashrc). It can shadow our packages at build and run time, so warn loudly.
check_foreign_workspaces() {
    local ros_prefix="/opt/ros/${ROS_DISTRO}"
    local found=0

    IFS=':' read -ra _prefixes <<< "${AMENT_PREFIX_PATH:-}"
    for prefix in "${_prefixes[@]}"; do
        [ -z "${prefix}" ] && continue
        case "${prefix}" in
            "${ros_prefix}"|"${INSTALL_DIR}"|"${INSTALL_DIR}"/*) continue ;;
        esac
        if [ "${found}" -eq 0 ]; then
            log_warn "Foreign ROS2 workspace(s) detected in AMENT_PREFIX_PATH:"
            found=1
        fi
        log_warn "  - ${prefix}"
    done

    if [ "${found}" -eq 1 ]; then
        log_warn "These overlays can shadow this workspace's packages (e.g. drdds)"
        log_warn "at build and runtime. If they are obsolete, remove the matching"
        log_warn "'source .../install/setup.bash' line from ~/.bashrc and re-run."
    fi
}

# ===== Step 2: build =====
step2_build() {
    log_step "Step 2/3: Build (colcon build --packages-up-to ${SERVICE_NAME})"

    cd "${WS_DIR}"
    check_foreign_workspaces
    source_env "${ROS_SETUP}"

    log_info "Removing previous build artifacts (build / install / log)..."
    rm -rf build install log

    log_info "Starting build..."
    echo "--------------------------------------------------------------"

    colcon build \
        --packages-up-to "${SERVICE_NAME}" \
        --allow-overriding drdds lite3_transfer lite3_sdk_service \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    echo "--------------------------------------------------------------"

    # Verify executable
    if [ ! -f "${EXEC_PATH}" ]; then
        log_error "Executable not found after build: ${EXEC_PATH}"
        log_error "Check the build logs above."
        exit 1
    fi

    log_done "Build succeeded. Executable: ${EXEC_PATH}"
}

# ===== Step 3: configure systemd auto-start =====
step3_setup_systemd() {
    log_step "Step 3/3: Configure systemd auto-start (only ${SERVICE_NAME}.service)"

    local service_file="/etc/systemd/system/${SERVICE_NAME}.service"

    log_info "Generating service file: ${service_file}"

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
                         exec ros2 run lite3_sdk_service sdk_service'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
SERVICEEOF

    log_info "Reloading systemd daemon..."
    sudo systemctl daemon-reload

    log_info "Enabling auto-start..."
    sudo systemctl enable "${SERVICE_NAME}"

    log_info "Starting service..."
    sudo systemctl restart "${SERVICE_NAME}"

    # Wait briefly, then check service state
    sleep 2
    if systemctl is-active --quiet "${SERVICE_NAME}"; then
        log_done "Service is running (active)."
    else
        log_warn "Service may not have started correctly. Check logs:"
        log_warn "  sudo journalctl -u ${SERVICE_NAME} -n 30"
    fi
}

# ===== Print completion message =====
print_finish() {
    cat <<EOF

${BOLD}${GREEN}====================================================
  Deployment complete!
====================================================${NC}
The only enabled systemd service:
  ${SERVICE_NAME}.service

Runtime model:
  - ${SERVICE_NAME} stays running
  - lite3_transfer is not configured as a standalone systemd service
  - lite3_transfer is started and stopped on demand by UDP "on"/"off" commands

Service management commands:
  Status:      sudo systemctl status ${SERVICE_NAME}
  Logs:        sudo journalctl -u ${SERVICE_NAME} -f
  Restart:     sudo systemctl restart ${SERVICE_NAME}
  Stop:        sudo systemctl stop ${SERVICE_NAME}
  Disable:     sudo systemctl disable ${SERVICE_NAME}

UDP control commands (port 12122):
  Start transfer:    echo "on"  | nc -u 127.0.0.1 12122
  Stop transfer:     echo "off" | nc -u 127.0.0.1 12122
  Set publish rate:  echo "50"  | nc -u 127.0.0.1 12122
${BOLD}${GREEN}====================================================${NC}
EOF
}

# ===== Main flow =====
main() {
    print_summary
    step1_check_env
    step2_build
    step3_setup_systemd
    print_finish
}

main "$@"
