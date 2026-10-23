# Builds kaliru, runs the unit tests and every parity check, prints a summary.
# Takes ~10 min. usage: check_all.sh
T=$(dirname "$(readlink -f "$0")")
bash $T/build.sh test
echo "== msgproto: parsedump";   python3 $T/parsedump_cmp.py --corrupt 10 | tail -1
echo "== msgproto: traces";      bash $T/trace_check.sh | tail -1
echo "== configfile";            bash $T/config_cmp.sh | tail -1
echo "== SAVE_CONFIG";           bash $T/save_cmp.sh | tail -1
echo "== pins";                  bash $T/pins_cmp.sh
echo "== reactor";               bash $T/reactor_cmp.sh | tail -1
echo "== serialqueue";           bash $T/sq_cmp.sh | tail -1
echo "== queuelogger";           bash $T/ql_cmp.sh
