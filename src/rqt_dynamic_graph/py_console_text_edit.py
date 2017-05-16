# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Dorian Scholz
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
from code import InteractiveInterpreter
from exceptions import SystemExit

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, Signal

from qt_gui_py_common.console_text_edit import ConsoleTextEdit

import roslib; roslib.load_manifest('rqt_dynamic_graph')
import rospy

import dynamic_graph_bridge.srv
import dynamic_graph_bridge_msgs.srv

class PyConsoleTextEdit(ConsoleTextEdit):
    _color_stdin = Qt.darkGreen
    _multi_line_char = ':'
    _multi_line_indent = '    '
    _prompt = ('>>> ', '... ')  # prompt for single and multi line
    exit = Signal()

    def __init__(self, parent=None):
        super(PyConsoleTextEdit, self).__init__(parent)

        self.cache = ""
        self._client = rospy.ServiceProxy(
            'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)

        self._comment_writer.write('Python %s on %s\n' % (sys.version.replace('\n', ''), sys.platform))
        self._comment_writer.write('Qt bindings: %s version %s\n' % (QT_BINDING, QT_BINDING_VERSION))

        self._add_prompt()

    def update_interpreter_locals(self, newLocals):
        pass

    def _exec_code(self, code):
        try:
            self._runcode(code)
        except SystemExit:  # catch sys.exit() calls, so they don't close the whole gui
            self.exit.emit()

    def _runcode(self, code, retry = True):
        self.cache += code + "\n"
        source = self.cache[:-1]
        self.cache = ""
        if source != "":
            try:
                if not self._client:
                    if not retry:
                        print("Connection to remote server lost. Reconnecting...")
                    self._client = rospy.ServiceProxy(
                        'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
                response = self._client(str(source))
                if response.standardoutput != "":
                    print(response.standardoutput[:-1])
                if response.standarderror != "":
                    print(response.standarderror[:-1])
                elif response.result != "None":
                    print(response.result)
            except rospy.ServiceException, e:
                print("Connection to remote server lost. Reconnecting...")
                self._client = rospy.ServiceProxy(
                    'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
                if retry:
                    self.cache = source
                    self._runcode(code, False)
                else:
                    print("Failed to connect. Is Stack of Tasks running?")
