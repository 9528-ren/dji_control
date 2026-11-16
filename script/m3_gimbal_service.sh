#!/bin/bash
# 用法: ./script/m3_gimbal_service.sh start|stop|restart|run|status
#   start   后台运行
#   stop    停止
#   restart 重启
#   run     前台运行(调试用, Ctrl+C 退出)
#   status  查看运行状态
ROOT_PWD=$(cd "$(dirname "$0")/.." && pwd)
APP=${ROOT_PWD}/build/m3_widget_gimbal
CFG=cfg/m3_gimbal.cfg
LOG=log/m3_widget_gimbal.log
# 进程名在 /proc 中被截断为 15 个字符
PROC_NAME=m3_widget_gimba

cd ${ROOT_PWD}
ulimit -n 4096


stop_conflict() {
  # 系统自带的云台 demo 会占用 PSDK 通道
  dji_app_ctl stop psdk-gimbal-demo >/dev/null 2>&1
}

stop_app() {
  if pgrep -x ${PROC_NAME} >/dev/null; then
    pkill -INT -x ${PROC_NAME}
    for i in 1 2 3 4 5; do
      pgrep -x ${PROC_NAME} >/dev/null || break
      sleep 1
    done
    pgrep -x ${PROC_NAME} >/dev/null && pkill -9 -x ${PROC_NAME}
    echo "m3_widget_gimbal stopped"
  fi
}

check_app() {
  if [ ! -x ${APP} ]; then
    echo "${APP} not found, run ./build.sh first"
    exit 1
  fi
}

case "$1" in
  start)
    check_app
    if pgrep -x ${PROC_NAME} >/dev/null; then
      echo "m3_widget_gimbal already running"
      exit 0
    fi
    stop_conflict
    mkdir -p log
    nohup ${APP} ${CFG} ${LOG} >log/stdout.out 2>&1 &
    sleep 1
    pgrep -x ${PROC_NAME} >/dev/null && echo "m3_widget_gimbal start, log: ${ROOT_PWD}/${LOG}" \
      || { echo "m3_widget_gimbal start failed, see log/stdout.out"; exit 1; }
    ;;
  stop)
    stop_app
    ;;
  restart)
    stop_app
    sleep 1
    $0 start
    ;;
  run)
    check_app
    stop_app
    stop_conflict
    exec ${APP} ${CFG} ${LOG}
    ;;
  status)
    pgrep -a -x ${PROC_NAME} || echo "m3_widget_gimbal not running"
    ;;
  *)
    echo "Usage: $0 start|stop|restart|run|status"
    exit 1
    ;;
esac
