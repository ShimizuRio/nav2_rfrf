def dprint(msg, node, lv: 1):
    # debug print
    # msg: message for print
    # node: has "debug_level" attribute
    # lv : debug level
    if hasattr(node, "debug_level"):
        debug_level = node.debug_level
    else:
        debug_level = 1
    if debug_level <= lv:
        print(f"{msg}")


def eq_find(lst, x):
    return [i for i, _x in enumerate(lst) if _x == x]


def neq_find(lst, x):
    return [i for i, _x in enumerate(lst) if _x != x]
