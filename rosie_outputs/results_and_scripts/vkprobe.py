import ctypes, ctypes.util, glob
lib = ctypes.util.find_library("vulkan") or (glob.glob("/isaac-sim/**/libvulkan.so.1", recursive=True) or ["libvulkan.so.1"])[0]
vk = ctypes.CDLL(lib)
class ICI(ctypes.Structure):
    _fields_ = [("sType", ctypes.c_int), ("pNext", ctypes.c_void_p), ("flags", ctypes.c_uint),
                ("pApplicationInfo", ctypes.c_void_p), ("enabledLayerCount", ctypes.c_uint),
                ("ppEnabledLayerNames", ctypes.c_void_p), ("enabledExtensionCount", ctypes.c_uint),
                ("ppEnabledExtensionNames", ctypes.c_void_p)]
inst = ctypes.c_void_p()
r = vk.vkCreateInstance(ctypes.byref(ICI(1, None, 0, None, 0, None, 0, None)), None, ctypes.byref(inst))
print("loader", lib, "| vkCreateInstance ->", r, "(0 = OK, -9 = INCOMPATIBLE_DRIVER)")
if r == 0:
    n = ctypes.c_uint32()
    vk.vkEnumeratePhysicalDevices(inst, ctypes.byref(n), None)
    print("physical GPUs visible to Vulkan:", n.value)
