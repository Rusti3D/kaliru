# Waits until a detached job ends, then prints the tail of its log
# usage: wait_for.sh <log file> <process pattern> [lines]
# (own pid excluded: our command line contains the pattern too)
while pgrep -f "$2" | grep -vqx $$; do sleep 10; done
tail -${3:-40} "$1"
