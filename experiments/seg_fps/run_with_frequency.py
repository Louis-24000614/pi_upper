"""保留旧实验启动命令；正式启动和实验使用同一套调频/恢复保护。"""
from road_follow.frequency_runtime import main

if __name__ == '__main__':
    raise SystemExit(main())
