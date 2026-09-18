"""RK3588 刀具识别模块：固定预处理、DINOv3 embedding 与模板检索。"""

__all__ = ["KnifeRecognizer"]


def __getattr__(name: str):
    """延迟加载板端引擎，使模板完整性测试不依赖OpenCV和RKNN。"""
    if name == "KnifeRecognizer":
        from .engine import KnifeRecognizer

        return KnifeRecognizer
    raise AttributeError(name)
