# Rebuilds the reference c_helper.so like klippy/chelper/__init__.py does,
# plus -ffp-contract=off: with -march=native gcc fuses a*b+c into FMA
# instructions, which makes the reference's last float bits depend on the
# build host. The Rust port follows the C source's plain IEEE operations.
# usage (reference venv, from ~/rk/kalico):
#   .venv/bin/python /mnt/d/RustyKlipper/kaliru/tools/build_ref_chelper.py
import os
import sys

sys.path.insert(0, ".")
from klippy import chelper

srcdir = os.path.dirname(os.path.realpath(chelper.__file__))
srcfiles = chelper.get_abs_files(srcdir, chelper.SOURCE_FILES)
destlib = chelper.get_abs_files(srcdir, [chelper.DEST_LIB])[0]
flags = chelper.NATIVE_FLAGS + " -ffp-contract=off"
cmd = "%s %s %s" % (chelper.GCC_CMD, flags, chelper.COMPILE_ARGS)
chelper.do_build_code(cmd % (destlib, " ".join(srcfiles)))
print("built", destlib)
