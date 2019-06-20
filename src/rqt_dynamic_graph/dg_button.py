import os

from std_srvs.srv import Empty
import rospy

from python_qt_binding import QT_BINDING_VERSION
if float(QT_BINDING_VERSION.split(".")[0]) < 5:
    # for retro-compatibility
    from python_qt_binding.QtGui import QPushButton, QPainter, QIcon
else:
    # for compatibility with pyqt5 or higher
    from python_qt_binding.QtWidgets import QPushButton
    from python_qt_binding.QtGui import QPainter, QIcon
from python_qt_binding.QtCore import QSize


class DynamicGraphButton(QPushButton):
    def __init__(self, parent=None):
        super(DynamicGraphButton, self).__init__(parent)
        this_dir, this_filename = os.path.split(__file__)
        stop_button_path = os.path.join(this_dir, "play_button.png")
        play_button_path = os.path.join(this_dir, "stop_button.png")
        finished_button_path = os.path.join(this_dir, "finished_button.png")
        self.icon_play = QIcon(stop_button_path)
        self.icon_stop = QIcon(play_button_path)
        self.icon_finished = QIcon(finished_button_path)

        self.setIcon(self.icon_play)

        self.pressed.connect(self.update)
        self.released.connect(self.update)
        self.clicked.connect(self.update_on_click)

        # states = ["init", "running", "stop"]
        self.state = "init"

        self._start_dg_client = self._get_start_dg_client()
        self._stop_dg_client = self._get_stop_dg_client()

        self.setMaximumWidth(100)
        self.setIconSize(QSize(0.8 * 100, 0.8 * 100))

    def update_on_click(self):
        if self.state == "init":
            self.setIcon(self.icon_stop)
            self._call_start_dg()
            self.state = "running"

        elif self.state == "running":
            self.setIcon(self.icon_finished)
            self._call_stop_dg()
            self.state = "stop"
        else:
            pass

    def enterEvent(self, event):
        self.update()

    def leaveEvent(self, event):
        self.update()

    def sizeHint(self):
        return QSize(100, 100)

    def _get_stop_dg_client(self):
        return rospy.ServiceProxy("/dynamic_graph/stop_dynamic_graph", Empty,
                                  True)

    def _get_start_dg_client(self):
        return rospy.ServiceProxy("/dynamic_graph/start_dynamic_graph", Empty,
                                  True)

    def _call_start_dg(self):
        try:
            if not self._start_dg_client:
                print("Connection to remote server lost. Reconnecting...")
                self._start_dg_client = self._get_start_dg_client()
            self._start_dg_client()
        except rospy.ServiceException, e:
            print("Connection to remote server lost. "
                  "Retry to start the dynamic graph...")
            self._start_dg_client = self._get_start_dg_client()
            self._start_dg_client()

    def _call_stop_dg(self):
        try:
            if not self._stop_dg_client:
                print("Connection to remote server lost. Reconnecting...")
                self._stop_dg_client = self._get_stop_dg_client()
            self._stop_dg_client()
        except rospy.ServiceException, e:
            print("Connection to remote server lost. "
                  "Retry to stop the dynamic graph...")
            self._stop_dg_client = self._get_stop_dg_client()
            self._stop_dg_client()
