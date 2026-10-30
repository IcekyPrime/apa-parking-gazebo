import ast
import sys

src = open(sys.argv[1]).read()
ast.parse(src)
sys.path.insert(0, os.path.expanduser('~/apa_ws/src/parking_planner'))
from parking_planner.controller import PathFollower  # noqa: E402
import os

f = PathFollower()
print('syntax OK, ref_ahead =', f.ref_ahead)
