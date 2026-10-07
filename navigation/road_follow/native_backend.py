"""实验用 C API 适配层；每个 context 独立，返回数组拷贝拥有独立生命周期。"""
import ctypes as C
from pathlib import Path
import time
import numpy as np


class NativeRknnSession:
    def __init__(self, model: Path, core: int, zero_input: bool):
        path = Path(__file__).with_name("native") / "build" / "libroad_rknn.so"
        self.lib = C.CDLL(str(path))
        lib = self.lib
        lib.road_rknn_create.argtypes = [C.c_char_p, C.c_int, C.c_int]
        lib.road_rknn_create.restype = C.c_void_p
        lib.road_rknn_error.restype = C.c_char_p
        lib.road_rknn_shape.argtypes = [C.c_void_p, C.c_int, C.POINTER(C.c_uint)]
        lib.road_rknn_output.argtypes = [C.c_void_p, C.c_int]
        lib.road_rknn_output.restype = C.POINTER(C.c_float)
        lib.road_rknn_infer.argtypes = [C.c_void_p, C.c_void_p, C.c_uint]
        lib.road_rknn_times.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
        lib.road_rknn_times.restype = None
        lib.road_rknn_destroy.argtypes = [C.c_void_p]
        lib.road_rknn_destroy.restype = None
        self.handle = lib.road_rknn_create(str(model).encode(), core, zero_input)
        if not self.handle:
            raise RuntimeError(lib.road_rknn_error().decode())
        self.shapes = []
        for index in range(2):
            dims = (C.c_uint*8)()
            rank = lib.road_rknn_shape(self.handle, index, dims)
            if rank <= 0:
                self.release()
                raise RuntimeError("无法查询输出 shape")
            self.shapes.append(tuple(dims[:rank]))
        self.last_timings = {}

    def inference(self, inputs, data_format):
        if not self.handle:
            raise RuntimeError("原生 RKNN context 已释放")
        if data_format != ["nhwc"] or len(inputs) != 1:
            raise ValueError("原生道路后端只支持一个 NHWC 输入")
        tensor = inputs[0]
        if tensor.shape != (1,640,640,3) or tensor.dtype != np.uint8 or not tensor.flags.c_contiguous:
            raise ValueError("原生输入必须是连续 uint8 1×640×640×3")
        if self.lib.road_rknn_infer(self.handle, tensor.ctypes.data, tensor.nbytes) != 0:
            raise RuntimeError(self.lib.road_rknn_error().decode())
        times = (C.c_double*3)()
        self.lib.road_rknn_times(self.handle, times)
        self.last_timings = dict(zip(["input_io_ms", "run_ms", "output_io_ms"], times))
        # C 内部 float 缓冲下一次推理会复用，必须在交给调用者前复制。
        began = time.perf_counter()
        outputs = [np.ctypeslib.as_array(self.lib.road_rknn_output(self.handle,i),
                                     shape=(int(np.prod(shape)),)).reshape(shape).copy()
                for i,shape in enumerate(self.shapes)]
        self.last_timings["python_output_copy_ms"] = (time.perf_counter()-began)*1000
        return outputs

    def release(self):
        if self.handle:
            self.lib.road_rknn_destroy(self.handle)
            self.handle = None
