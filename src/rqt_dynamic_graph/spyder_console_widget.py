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

from python_qt_binding.QtGui import QFont

from spyderlib.widgets.internalshell import InternalShell
from spyderlib.utils.module_completion import moduleCompletion

import roslib; roslib.load_manifest('rqt_dynamic_graph')
import rospy

import dynamic_graph_bridge.srv

class SpyderConsoleWidget(InternalShell):
    def __init__(self, context=None):
        my_locals = {
            'context': context
        }
        super(SpyderConsoleWidget, self).__init__(namespace=my_locals)

        self.cache = ""
        self._client = rospy.ServiceProxy(
            'run_command', dynamic_graph_bridge.srv.RunCommand, True)

        self.setObjectName('SpyderConsoleWidget')
        self.set_pythonshell_font(QFont('Mono'))
        self.interpreter.restore_stds()

    def get_module_completion(self, objtxt):
        """Return module completion list associated to object name"""
        return moduleCompletion(objtxt)

    def run_command(self, code):
        self.interpreter.redirect_stds()
        super(SpyderConsoleWidget, self).run_command("")
        self._runcode(code)
        self.flush()
        self.interpreter.restore_stds()

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
                        'run_command', dynamic_graph_bridge.srv.RunCommand, True)
                response = self._client(str(source))
                if response.stdout != "":
                    print(response.stdout[:-1])
                if response.stderr != "":
                    print(response.stderr[:-1])
                elif response.result != "None":
                    print(response.result)
                print("\n")
            except rospy.ServiceException, e:
                print("Connection to remote server lost. Reconnecting...")
                self._client = rospy.ServiceProxy(
                    'run_command', dynamic_graph_bridge.srv.RunCommand, True)
                if retry:
                    self.cache = source
                    self._runcode(code, False)
                else:
                    print("Failed to connect. Is Stack of Tasks running?")

    def shutdown(self):
        self.exit_interpreter()
