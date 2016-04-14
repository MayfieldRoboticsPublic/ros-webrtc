import re

import rosgraph


def normalize_ros_name(name):
    return re.sub(r'[^\w_\/]', '', name)


def join_ros_names(*names):
    return reduce(
        rosgraph.names.ns_join,
        map(normalize_ros_name, (n for n in names if n))
    )
