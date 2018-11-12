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
import os
from exceptions import SystemExit

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, Signal

from qt_gui_py_common.console_text_edit import ConsoleTextEdit

import roslib; roslib.load_manifest('rqt_dynamic_graph')
import rospy

from dynamic_graph_manager.ros.dgcompleter import DGCompleter
try:
    from dynamic_graph_manager.srv import RunCommand as ros_srv_RunCommand
    run_command_service_name = '/dynamic_graph/run_python_command'
except:
    from dynamic_graph_bridge_msgs.srv import RunCommand as ros_srv_RunCommand
    run_command_service_name = 'run_command'

# manages python history
import pickle
# manages the log and time stamps
from datetime import datetime
import time

class PyConsoleTextEdit(ConsoleTextEdit):
    _color_stdin = Qt.darkGreen
    _multi_line_char = ':'
    _multi_line_indent = '    '
    _prompt = ('>>> ', '... ')  # prompt for single and multi line
    exit = Signal()

    def __init__(self, parent=None):
        super(PyConsoleTextEdit, self).__init__(parent)

        self.cache = ""
        self._client = self._get_RunCommand_client()

        self._comment_writer.write(
            'Python %s on %s\n' % (sys.version.replace('\n', ''), sys.platform))
        self._comment_writer.write(
            'Qt bindings: %s version %s\n' % (QT_BINDING, QT_BINDING_VERSION))

        self._add_prompt()

        self._init_log_and_history()

        # for autocompletion
        self.completer = DGCompleter(self._client)

    def keyPressEvent(self, event):
        prompt_length = len(self._prompt[self._multi_line])
        block_length = self.document().lastBlock().length()
        document_length = self.document().characterCount()
        line_start = document_length - block_length
        prompt_position = line_start + prompt_length

        # only handle keys if cursor is in the last line
        if self.textCursor().position() >= prompt_position:
            if event.key() == Qt.Key_Tab:
                last_line = self.document().lastBlock().text()[prompt_length:]
                possible_responses = []
                response = ""
                nb_response = 0
                while True:
                    response = self.completer.complete(last_line, nb_response)
                    if response is not None:
                        possible_responses.append(response)
                        nb_response += 1
                    else:
                        break

                if len(possible_responses) == 1:
                    self._clear_current_line(clear_prompt=False)
                    self._comment_writer.write(possible_responses[0])
                elif len(possible_responses) > 1:
                    self._stdout_list(possible_responses)
                    self._add_prompt()
                    self._clear_current_line(clear_prompt=False)
                    new_line = self._common_prefix(possible_responses)
                    self._comment_writer.write(new_line)

                return None

        # allow all other key events
        super(PyConsoleTextEdit, self).keyPressEvent(event)

    def update_interpreter_locals(self, newLocals):
        pass

    def _stdout_list(self, my_list):
        self._stdout.write('\n[')
        for el in my_list:
            self._stdout.write(el + ', ')
        self._stdout.write(']\n')

    # Return the longest prefix of all list elements.
    def _common_prefix(self, m):
        "Given a list of pathnames, returns the longest common leading component"
        if not m: return ''
        s1 = min(m)
        s2 = max(m)
        for i, c in enumerate(s1):
            if c != s2[i]:
                return s1[:i]
        return s1

    def _get_dir(self, objtxt):
        """Return dir(object)"""
        if(objtxt is None):
            return;
        source = 'dir('+objtxt+')';
        response = self._runcode(source);
        if((response is None) or (response.result is None)):
            return;
        res = response.result[2:-2].split("', '");

        # check whether the object is an entity
        if('signals' in res):
            source = '[s.name.split("::")[-1] for s in '+objtxt+'.signals()]';
            response = self._runcode(source);
            if(response != None):
                res += response.result[2:-2].split("', '");
            source = objtxt+'.commands()';
            response = self._runcode(source);
            if(response != None):
                res += response.result[2:-2].split("', '");
        return res;

    def _get_globals_keys(self):
        """Return shell globarls() keys"""
        response = self._runcode('globals().keys()');
        if(response is None):
            return;
        return response.result[2:-2].split("', '");

    def _get__doc__(self, objtxt):
        """Get object __doc__"""
        if(objtxt is None):
            return;
        source = 'help('+objtxt+')';
        response = self._runcode(source);
        if(response is None):
            return;
        return response.standardoutput;

    def _get_RunCommand_client(self):
        return rospy.ServiceProxy(run_command_service_name,
                                  ros_srv_RunCommand, True)

    def _init_log_and_history(self):
        self.log_path = "%s/.rqt_dynamic_graph/" % os.environ["HOME"]
        self.python_hist_file = self.log_path + 'python_history.pkl'
        self.log_file = (self.log_path +
                         datetime.now().strftime("%y_%m_%d__%H_%M")+'.log')
        # open text file for logging
        try:
            if(not os.path.exists(self.log_path)):
                os.mkdir(self.log_path)
            # we get the standard session and load it
            if os.path.exists(self.python_hist_file):
                self._history = pickle.load(open(self.python_hist_file, 'r'))
            else:
                self._history = []
            self.log = open(self.log_file, 'a')
            self.log_time = time.time()
        except Exception as e:
            print "ERROR: Could not open log or history file!"
            print e
            self.log = None

    def _exec_code(self, code):
        try:
            self._print_response(self._runcode(code))
        # catch sys.exit() calls, so they don't close the whole gui
        except SystemExit:
            pickle.dump(self._history, open(self.python_hist_file, 'w'))
            self.exit.emit()

    def _print_response(self, response):
        if response.standardoutput != "":
            self._stdout.write(response.standardoutput[:-1] + '\n')
        if response.standarderror != "":
            self._stderr.write(response.standarderror[:-1] + '\n')
        elif response.result != "None":
            self._stdout.write(response.result + '\n')

    def _runcode(self, code, retry = True):
        self.cache += code + "\n"
        source = self.cache[:-1]
        self.cache = ""
        if source != "":
            try:
                if not self._client:
                    if not retry:
                        print("Connection to remote server lost. " +
                              "Reconnecting...")
                    self._client = self._get_RunCommand_client()
                response = self._client(str(source))
                # not very clever but I cannot detect when the gui is closing...
                pickle.dump(self._history, open(self.python_hist_file, 'w'))
                return response
            except rospy.ServiceException, e:
                print("Connection to remote server lost. Reconnecting...")
                self._client = self._get_RunCommand_client()
                if retry:
                    self.cache = source
                    self._runcode(code, False)
                else:
                    print("Failed to connect. Is Stack of Tasks running?")
