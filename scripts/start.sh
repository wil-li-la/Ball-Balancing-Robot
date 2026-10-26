#!/bin/sh
# Start and stop the long-running helpers on the Pi. Logs go to ../logs/.
#   ./start.sh camd        camera broker (camd.py), needed by everything else
#   ./start.sh dashboard   live dashboard on 127.0.0.1:8000 (from the Mac: ssh -N -L 8000:127.0.0.1:8000 xiangen@pi4.local)
#   ./start.sh balancer ARGS...   ball_balancer.py in the background, e.g. ./start.sh balancer --pid --camera sonix
#   ./start.sh all | stop | status
cd "$(dirname "$0")" || exit 1
mkdir -p ../logs
CAMERA=${CAMERA:-sonix}
running() { pgrep -f "python3 -u $1.py" > /dev/null; }
start() {
    if running "$1"; then echo "$1 already running"; return; fi
    setsid -f nohup python3 -u "$1.py" "$2" "$3" > "../logs/$1.log" 2>&1 < /dev/null
    sleep 2
    if running "$1"; then echo "$1 started"; else echo "$1 FAILED:"; tail -5 "../logs/$1.log"; fi
}
case "$1" in
    camd)       start camd --camera "$CAMERA"; sudo renice -n -10 -p $(pgrep -f "python3 -u camd.py") > /dev/null ;;   # the capture must not be starved
    dashboard)  start dashboard --camera "$CAMERA" ;;
    all)        "$0" camd; start dashboard --camera "$CAMERA" ;;
    balancer)   shift
                if running ball_balancer; then pkill -TERM -f "python3 -u [b]all_balancer.py"; sleep 2; fi
                setsid -f nohup python3 -u ball_balancer.py "$@" > ../logs/balancer.log 2>&1 < /dev/null
                sleep 3; if running ball_balancer; then echo "balancer started: $*"; else echo "balancer FAILED:"; tail -5 ../logs/balancer.log; fi ;;
    stop)       pkill -TERM -f "python3 -u [b]all_balancer.py"; sleep 1; pkill -f "python3 -u [d]ashboard.py"; pkill -f "python3 -u [c]amd.py"; echo stopped ;;
    status)     for p in camd dashboard ball_balancer; do if running $p; then echo "$p running"; else echo "$p stopped"; fi; done ;;
    *)          echo "usage: $0 camd|dashboard|balancer ARGS|all|stop|status"; exit 1 ;;
esac
