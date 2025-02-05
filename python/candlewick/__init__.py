from .pycandlewick import *
from .pycandlewick import __version__


def _process():
    import sys
    import inspect
    from . import pycandlewick

    lib_name = "candlewick"
    submodules = inspect.getmembers(pycandlewick, inspect.ismodule)
    for mod_info in submodules:
        mod_name = "{}.{}".format(lib_name, mod_info[0])
        sys.modules[mod_name] = mod_info[1]
        mod_info[1].__file__ = pycandlewick.__file__
        mod_info[1].__name__ = mod_name


_process()
