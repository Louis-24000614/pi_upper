"""RoboCup 公共安全赛项侦查机器人上位机入口。"""

import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from main_window import MainWindow


def load_style(app: QApplication) -> None:
    style_path = Path(__file__).with_name("style.qss")
    app.setStyleSheet(style_path.read_text(encoding="utf-8"))


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("RoboCup 侦查机器人上位机")
    load_style(app)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
