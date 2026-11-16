#!/bin/bash
PROG=/home/dji/m3_widget_gimbal_v2/build/m3_widget_gimbal
DIR=/home/dji/m3_widget_gimbal_v2
LOG=log/m3_widget_gimbal.log
PIDFILE=$DIR/run.pid

wait_usb() {
    for i in $(seq 1 30); do
        if lsusb 2>/dev/null | grep -qi "dji\|2ca3\|0bda.*dji"; then
            return 0
        fi
        # Also check if /dev/dji* exists
        if ls /dev/dji* 2>/dev/null | head -1 > /dev/null 2>&1; then
            return 0
        fi
        sleep 2
    done
    return 1
}

start() {
    if [ -f "$PIDFILE" ] && kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
        echo "already running, pid=$(cat $PIDFILE)"
        return 0
    fi
    cd "$DIR"
    echo "waiting for drone USB..."
    wait_usb
    echo "starting m3_widget_gimbal..."
    nohup $PROG cfg/m3_gimbal.cfg "$LOG" >> log/stdout.out 2>&1 &
    echo $! > "$PIDFILE"
    echo "started, pid=$!"
}

stop() {
    if [ -f "$PIDFILE" ]; then
        PID=$(cat "$PIDFILE")
        kill -9 "$PID" 2>/dev/null && echo "stopped pid=$PID" || echo "pid $PID not found"
        rm -f "$PIDFILE"
    else
        pkill -9 -f "$PROG" 2>/dev/null && echo "stopped" || echo "not running"
    fi
}

case "$1" in
  start)   start ;;
  stop)    stop ;;
  restart) stop; sleep 3; start ;;
  status)
    if [ -f "$PIDFILE" ] && kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
        echo "running, pid=$(cat $PIDFILE)"
    else
        echo "not running"
    fi
    ;;
  *) echo "Usage: $0 {start|stop|restart|status}" ;;
esac

