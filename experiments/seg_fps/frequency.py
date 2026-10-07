"""兼容旧实验入口；调频逻辑已移入正式道路运行模块，避免维护两份实现。"""
from pathlib import Path
import sys

# 此文件也会被旧脚本通过绝对路径以 sudo 执行；sudo 会清除 PYTHONPATH，
# 因而必须从本仓库确定 navigation 路径，不能依赖调用者当前目录。
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'navigation'))
from road_follow.frequency import (apply, discover, main, restore, set_domain, snapshot, validate)

if __name__ == '__main__':
    main()
