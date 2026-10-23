# Runs the Python reference over all test/klippy cases into ~/rk/<name>.
# Runs in the foreground: WSL stops the distro (killing detached jobs) once no
# wsl.exe session is attached, so call this from a long-lived session.
# usage: run_ref.sh <name> [PYTHONHASHSEED]
cd "$(dirname "$0")"
PYTHONHASHSEED=${2:-0} python3 refrun.py --out ~/rk/$1 < /dev/null > ~/rk/$1.out 2>&1
tail -3 ~/rk/$1.out
