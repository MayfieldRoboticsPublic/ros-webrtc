import contextlib
import os
import shutil
import sys

import rostest


@contextlib.contextmanager
def ros_coverage():
    """
    https://github.com/ros/ros_comm/issues/558
    """
    coverage_mode = '--cov' in sys.argv
    if coverage_mode:
        sys.argv.remove('--cov')
        src = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.coveragerc')
        dst = os.path.join(os.getcwd(), '.coveragerc')
        if not os.path.exists(dst):
            shutil.copyfile(src, dst)
        rostest._start_coverage(['ros_webrtc'])
    try:
        yield
    finally:
        if coverage_mode:
            rostest._stop_coverage(['ros_webrtc'])
