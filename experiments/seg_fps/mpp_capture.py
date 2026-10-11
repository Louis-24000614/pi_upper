"""FFmpeg/OpenCV demuxes original AVI packets; MPP alone decodes MJPEG."""
import ctypes as C
from pathlib import Path
import time
import json
import subprocess
import cv2
import numpy as np
from experiments.seg_fps.frames import VideoFrames, Frame


class MppCapture:
    def __init__(self, path, rgb_output=False):
        if rgb_output:
            raise ValueError("direct MPP BGR output failed pixel validation; use validated NV12 conversion")
        self.path = str(path)
        metadata = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
            "stream=codec_name,color_range,color_space,pix_fmt", "-of", "json", self.path]))
        stream = metadata["streams"][0]
        if (stream.get("codec_name"), stream.get("color_range"), stream.get("color_space")) != (
                "mjpeg", "pc", "bt470bg"):
            raise ValueError(f"unsupported source color convention: {stream}")
        self.stream = stream
        self.cap = cv2.VideoCapture(self.path, cv2.CAP_FFMPEG)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.handle = None
        self.lib = C.CDLL(str(Path(__file__).parent / "build" / "libmpp_jpeg.so"))
        self.lib.road_mpp_create.argtypes = [C.c_int, C.c_int, C.c_int]
        self.lib.road_mpp_create.restype = C.c_void_p
        self.lib.road_mpp_error.restype = C.c_char_p
        self.lib.road_mpp_decode.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t, C.c_void_p,
                                           C.c_size_t, C.c_void_p, C.c_void_p]
        self.lib.road_mpp_destroy.argtypes = [C.c_void_p]
        self.samples = []
        self.last_info = None
        try:
            if not self.cap.isOpened() or not self.cap.set(cv2.CAP_PROP_FORMAT, -1):
                raise RuntimeError("FFmpeg raw packet demux unavailable")
            self.handle = self.lib.road_mpp_create(self.width, self.height, rgb_output)
            if not self.handle:
                raise RuntimeError(self.lib.road_mpp_error().decode())
            self.buffer = np.empty(((self.width + 15) // 16 * 16) *
                                   ((self.height + 15) // 16 * 16) * 4, np.uint8)
        except BaseException:
            self.release()
            raise

    def isOpened(self):
        return self.cap.isOpened() and bool(self.handle)

    def get(self, prop):
        return self.cap.get(prop)

    def set(self, prop, value):
        if prop != cv2.CAP_PROP_POS_FRAMES or value != 0:
            raise ValueError("MPP replay only supports rewind to the first frame")
        self.cap.release()
        self.cap = cv2.VideoCapture(self.path, cv2.CAP_FFMPEG)
        if not self.cap.set(cv2.CAP_PROP_FORMAT, -1):
            raise RuntimeError("failed to reopen original packet stream")
        return True

    def read(self):
        start = time.perf_counter()
        ok, packet = self.cap.read()
        if not ok:
            return False, None
        demux_ms = (time.perf_counter() - start) * 1000
        info = (C.c_int * 8)()
        times = (C.c_double * 3)()
        packet = np.ascontiguousarray(packet)
        if self.lib.road_mpp_decode(self.handle, packet.ctypes.data, packet.nbytes,
                                    self.buffer.ctypes.data, self.buffer.nbytes, info, times) != 0:
            raise RuntimeError(self.lib.road_mpp_error().decode())
        width, height, ws, hs, fmt, _, _, count = info
        begin = time.perf_counter()
        if fmt == 0:
            yuv = self.buffer[:count].reshape(hs * 3 // 2, ws)
            uv = yuv[hs:].reshape(hs // 2, ws // 2, 2)[:height // 2, :width // 2]
            chroma = cv2.resize(uv, (width, height), interpolation=cv2.INTER_NEAREST)
            full_range = np.empty((height, width, 3), np.uint8)
            full_range[:, :, 0] = yuv[:height, :width]
            full_range[:, :, 1] = chroma[:, :, 1]
            full_range[:, :, 2] = chroma[:, :, 0]
            # JPEG source is full-range BT.601, not OpenCV's limited-range NV12 convention.
            image = cv2.cvtColor(full_range, cv2.COLOR_YCrCb2BGR)
        else:
            image = self.buffer[:count].reshape(hs, ws, 3)[:height, :width].copy()
        self.last_info = list(info)
        self.samples.append({"demux_ms": demux_ms, "input_copy_ms": times[0],
                             "decode_sync_ms": times[1], "output_copy_ms": times[2],
                             "convert_ms": (time.perf_counter() - begin) * 1000,
                             "total_ms": (time.perf_counter() - start) * 1000})
        return True, image

    def release(self):
        if self.handle:
            self.lib.road_mpp_destroy(self.handle)
            self.handle = None
        self.cap.release()

    close = release


class MppFrames(VideoFrames):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.capture_resource = MppCapture(self.path)
        for _ in range(20):
            if not self.capture_resource.read()[0]:
                self.capture_resource.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self.capture_resource.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self.capture_resource.samples.clear()

    def _iterate(self):
        cap = self.capture_resource
        fps = cap.get(cv2.CAP_PROP_FPS)
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if fps <= 0 or count <= 0:
            cap.release()
            raise ValueError("AVI lacks valid FPS/frame count")
        start, sequence, decoded = time.monotonic(), 0, 0
        try:
            while not self.stop.is_set():
                t0 = time.monotonic()
                ok, image = cap.read()
                t1 = time.monotonic()
                if not ok:
                    if decoded < count:
                        raise RuntimeError(f"early MPP EOF {decoded}/{count}")
                    if not self.loop:
                        break
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    decoded = 0
                    continue
                decoded += 1
                self.decode_ms.append((t1 - t0) * 1000)
                due = start + sequence / fps / self.speed
                if self.realtime and self.stop.wait(max(0, due - time.monotonic())):
                    break
                yield Frame(sequence, sequence / fps, due if self.realtime else t0, t1, 0, image)
                sequence += 1
        finally:
            cap.release()

    def close(self):
        super().close()
        self.capture_resource.release()
