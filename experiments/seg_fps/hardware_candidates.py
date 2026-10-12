"""Experimental adapters only; never patch the production entry point."""
import ctypes as C
from pathlib import Path
import time
import cv2
import numpy as np
from road_follow.pipeline import BevProjector, make_ipm
from road_follow.segment_buffers import SegmentBuffers

ROOT = Path(__file__).parent


class OpenClProjector(BevProjector):
    def __init__(self):
        super().__init__()
        self.handle = None
        self.last_timings = {}
        self.samples = []
        self.lib = C.CDLL(str(ROOT / "build" / "libopencl_bev.so"))
        self.lib.road_cl_error.restype = C.c_char_p
        self.lib.road_cl_create.argtypes = [C.c_size_t, C.c_void_p, C.c_size_t]
        self.lib.road_cl_create.restype = C.c_void_p
        self.lib.road_cl_run.argtypes = [C.c_void_p, C.c_void_p, C.c_void_p, C.c_void_p]
        self.lib.road_cl_destroy.argtypes = [C.c_void_p]

    def project(self, mask, cfg):
        if mask.ndim != 2 or mask.dtype != np.uint8:
            raise ValueError("OpenCL BEV accepts only uint8 single-channel masks")
        key = (mask.shape, tuple(sorted((cfg.get("camera") or {}).items())),
               tuple(sorted((cfg.get("bev") or {}).items())))
        if key != self._key:
            self.close()
            self._ipm = make_ipm(cfg, mask.shape)
            # Use OpenCV's exact nearest-neighbor border/rounding convention once.
            # The GPU executes the pixel gather; it does not change IPM geometry.
            if mask.size >= 2**24:
                raise ValueError("source indices exceed exact float32 integer range")
            indices = np.arange(mask.size, dtype=np.float32).reshape(mask.shape)
            width, height = self._ipm.bev.size()
            mapping = cv2.warpPerspective(indices, self._ipm.H_img_to_bev, (width, height),
                                          flags=cv2.INTER_NEAREST, borderValue=-1).astype(np.int32)
            self.handle = self.lib.road_cl_create(mask.size, mapping.ctypes.data, mapping.size)
            if not self.handle:
                raise RuntimeError(self.lib.road_cl_error().decode())
            self.output_shape = mapping.shape
            self._key = key
        image = np.ascontiguousarray(mask)
        output = np.empty(self.output_shape, np.uint8)
        times = (C.c_double * 4)()
        started = time.perf_counter()
        if self.lib.road_cl_run(self.handle, image.ctypes.data, output.ctypes.data, times) != 0:
            raise RuntimeError(self.lib.road_cl_error().decode())
        self.last_timings = dict(zip(("upload_ms", "dispatch_sync_ms", "download_ms", "kernel_ms"), times))
        self.last_timings["total_ms"] = (time.perf_counter() - started) * 1000
        self.samples.append(dict(self.last_timings))
        return self._ipm, output

    def close(self):
        if self.handle:
            self.lib.road_cl_destroy(self.handle)
            self.handle = None
        self._key = None


class RgaBuffers(SegmentBuffers):
    def __init__(self):
        super().__init__()
        self.handle = None
        self.shape = None
        self.lib = C.CDLL(str(ROOT / "build" / "librga_dma_v4.so"))
        self.lib.road_rga_create.argtypes = [C.c_int, C.c_int]
        self.lib.road_rga_create.restype = C.c_void_p
        self.lib.road_rga_error.restype = C.c_char_p
        self.lib.road_rga_run.argtypes = [C.c_void_p, C.c_void_p, C.c_void_p,
                                        C.c_int, C.c_int, C.c_int, C.c_int]
        self.lib.road_rga_destroy.argtypes = [C.c_void_p]
        self.last_timings = {}

    def letterbox(self, bgr):
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError("RGA accepts only uint8 BGR frames")
        if bgr.shape != self.shape:
            self.close()
            height, width = bgr.shape[:2]
            self.handle = self.lib.road_rga_create(width, height)
            if not self.handle:
                raise RuntimeError(self.lib.road_rga_error().decode())
            self.shape = bgr.shape
        height, width = bgr.shape[:2]
        ratio = min(640 / height, 640 / width)
        rw, rh = round(width * ratio), round(height * ratio)
        left, top = (640 - rw) // 2, (640 - rh) // 2
        image = np.ascontiguousarray(bgr)
        start = time.perf_counter()
        if self.lib.road_rga_run(self.handle, image.ctypes.data, self.rgb.ctypes.data,
                                 rw, rh, left, top) != 0:
            raise RuntimeError(self.lib.road_rga_error().decode())
        self.last_timings = {"total_ms": (time.perf_counter() - start) * 1000}
        return self.rgb, ratio, left, top

    def close(self):
        if self.handle:
            self.lib.road_rga_destroy(self.handle)
            self.handle = None
        self.shape = None


def install(candidate):
    resources = []
    if candidate == "gpu-bev":
        from experiments.seg_fps import navigation, benchmark
        original = navigation.Navigation

        class GpuNavigation(original):
            def __init__(self, cfg, reuse_bev=False, lazy_raw=False):
                if not reuse_bev:
                    raise ValueError("GPU candidate requires the buffers/BEV baseline")
                super().__init__(cfg, reuse_bev, lazy_raw)
                self.projector = OpenClProjector()
                resources.append(self.projector)

        benchmark.Navigation = GpuNavigation
    elif candidate == "rga":
        from experiments.seg_fps import benchmark
        original = benchmark.RoadSegmenter

        class RgaSegmenter(original):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                if not self.reuse_buffers:
                    raise ValueError("RGA candidate requires the buffers baseline")
                self._buffers = RgaBuffers()
                resources.append(self._buffers)

        benchmark.RoadSegmenter = RgaSegmenter
    elif candidate in ("native-standard", "native-output"):
        from experiments.seg_fps import benchmark
        from road_follow.native_backend import NativeRknnSession
        original = benchmark.RoadSegmenter

        class OutputSegmenter(original):
            def _load(self):
                if self._session is None:
                    self._session = OutputSession(self.model_path, self.core_mask or 1,
                                                  3 if candidate == "native-output" else 1)
                return self._session

        benchmark.RoadSegmenter = OutputSegmenter
    elif candidate == "mpp-decode":
        from experiments.seg_fps import benchmark
        from experiments.seg_fps.mpp_capture import MppFrames

        class MeasuredMppFrames(MppFrames):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                resources.append(self.capture_resource)

        benchmark.VideoFrames = MeasuredMppFrames
    elif candidate in ("rga-copy", "rga-shared"):
        from experiments.seg_fps import benchmark
        original = benchmark.RoadSegmenter

        class SharedSegmenter(original):
            def _load(self):
                if self._session is None:
                    self._session = OutputSession(self.model_path, self.core_mask or 1, 1)
                    self._buffers = (SharedRgaBuffers(self._session) if candidate == "rga-shared"
                                     else RgaBuffers())
                    resources.append(self._buffers)
                return self._session

            def close(self):
                if self._buffers is not None:
                    self._buffers.close()
                super().close()

        benchmark.RoadSegmenter = SharedSegmenter
    elif candidate != "cpu":
        raise ValueError(f"unknown candidate {candidate}")
    return resources


class OutputSession:
    def __init__(self, model, core, flags):
        from road_follow.native_backend import NativeRknnSession
        # Initialize the same Python contract against an isolated experimental library.
        self.lib = C.CDLL(str(ROOT / "build" / "libnative_output_v3.so"))
        lib = self.lib
        lib.road_rknn_create.argtypes = [C.c_char_p, C.c_int, C.c_int]
        lib.road_rknn_create.restype = C.c_void_p
        lib.road_rknn_error.restype = C.c_char_p
        lib.road_rknn_shape.argtypes = [C.c_void_p, C.c_int, C.POINTER(C.c_uint)]
        lib.road_rknn_output.argtypes = [C.c_void_p, C.c_int]
        lib.road_rknn_output.restype = C.POINTER(C.c_float)
        lib.road_rknn_infer.argtypes = [C.c_void_p, C.c_void_p, C.c_uint]
        lib.road_rknn_times.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
        lib.road_rknn_destroy.argtypes = [C.c_void_p]
        lib.road_rknn_destroy.restype = None
        self.handle = lib.road_rknn_create(str(model).encode(), core, flags)
        if not self.handle:
            raise RuntimeError(lib.road_rknn_error().decode())
        self.shapes = []
        for i in range(2):
            dims = (C.c_uint * 8)()
            rank = lib.road_rknn_shape(self.handle, i, dims)
            if rank <= 0:
                self.release()
                raise RuntimeError("output shape query failed")
            self.shapes.append(tuple(dims[:rank]))
        self.last_timings = {}
        self._infer = NativeRknnSession.inference
        self._release = NativeRknnSession.release

    def inference(self, inputs, data_format):
        return self._infer(self, inputs, data_format)

    def release(self):
        return self._release(self)


class SharedRgaBuffers(RgaBuffers):
    def __init__(self, session):
        super().__init__()
        self.session = session
        lib = session.lib
        for name in ("input_fd", "input_size", "input_stride", "input_ptr", "mark_ready"):
            getattr(lib, "road_rknn_" + name).argtypes = [C.c_void_p]
        lib.road_rknn_input_ptr.restype = C.c_void_p
        lib.road_rknn_input_size.restype = C.c_uint
        lib.road_rknn_input_stride.restype = C.c_uint
        self.fd = lib.road_rknn_input_fd(session.handle)
        self.pointer = lib.road_rknn_input_ptr(session.handle)
        self.bytes = lib.road_rknn_input_size(session.handle)
        self.stride = lib.road_rknn_input_stride(session.handle)
        if not self.pointer or self.fd < 0 or self.stride != 640:
            raise ValueError("shared experimental path requires queried packed 640-wide input")
        raw = np.ctypeslib.as_array(C.cast(self.pointer, C.POINTER(C.c_uint8)), shape=(self.bytes,))
        self.rgb = raw[:640 * 640 * 3].reshape(640, 640, 3)
        self.lib.road_rga_run_shared.argtypes = [C.c_void_p, C.c_void_p, C.c_void_p,
                                               C.c_int, C.c_uint, C.c_uint,
                                               C.c_int, C.c_int, C.c_int, C.c_int]

    def letterbox(self, bgr):
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError("shared RGA requires uint8 BGR")
        if bgr.shape != self.shape:
            self.close()
            height, width = bgr.shape[:2]
            self.handle = self.lib.road_rga_create(width, height)
            if not self.handle:
                raise RuntimeError(self.lib.road_rga_error().decode())
            self.shape = bgr.shape
        height, width = bgr.shape[:2]
        ratio = min(640 / height, 640 / width)
        rw, rh = round(width * ratio), round(height * ratio)
        left, top = (640 - rw) // 2, (640 - rh) // 2
        image = np.ascontiguousarray(bgr)
        start = time.perf_counter()
        if self.lib.road_rga_run_shared(self.handle, image.ctypes.data, self.pointer,
                                        self.fd, self.bytes, self.stride, rw, rh, left, top) != 0:
            raise RuntimeError(self.lib.road_rga_error().decode())
        self.session.lib.road_rknn_mark_ready(self.session.handle)
        self.last_timings = {"total_ms": (time.perf_counter() - start) * 1000}
        return self.rgb, ratio, left, top
