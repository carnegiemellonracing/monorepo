import ctypes
import ctypes.utils
import os

can_id_path = os.path.join("stm32f413-drivers", "CMR", "include", "CMR", "can_ids.h")

XSULONG32 = ctypes.c_uint32
for name, value in XS_INFO.__dict__.items():
    if not name.startswith("__") and not callable(value):
        print(f"{name}: {value}")