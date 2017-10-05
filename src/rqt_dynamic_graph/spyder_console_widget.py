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
from spyderlib.utils.module_completion import module_completion as moduleCompletion

import roslib; roslib.load_manifest('rqt_dynamic_graph')
import rospy

import dynamic_graph_bridge.srv
import dynamic_graph_bridge_msgs.srv

from spyderlib.utils.dochelpers import getobjdir

from datetime import datetime
import os

class SpyderConsoleWidget(InternalShell):
    _multi_line_char = ':'
    _multi_line_indent = ''
    _prompt = ('>>> ', '... ')  # prompt for single and multi line

    def __init__(self, context=None):
        my_locals = {
            'context': context
        }
        super(SpyderConsoleWidget, self).__init__(namespace=my_locals)

        self.cache = ""
        self._client = rospy.ServiceProxy(
            'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)

        self.setObjectName('SpyderConsoleWidget')
        self.set_pythonshell_font(QFont('Mono'))
        self.interpreter.restore_stds()

        self._multi_line = False
        self._multi_line_level = 0
        self._command = ''

        # open text file for logging
        try:
            path = os.path.expanduser('~/.rqt_dynamic_graph/');
            if(not os.path.exists(path)):
                os.mkdir(path)
            self.log = open(path+datetime.now().strftime("%y_%m_%d__%H_%M")+'.log', 'a') 
            #print "Log file successfully open"
        except Exception as e:
            print "ERROR: Could not open log file!"
            print e
            self.log = None

    def get_module_completion(self, objtxt):
        """Return module completion list associated to object name"""
        # FIXME: This executes on the local machine, so it may suggest modules
        # that are not there on the robot!
        return moduleCompletion(objtxt)

    def get_dir(self, objtxt):
        """Return dir(object)"""
        if(objtxt is None):
            return;
        source = 'dir('+objtxt+')';
        response = self._runcode(source);
        if(response is None):
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

    def get_globals_keys(self):
        """Return shell globarls() keys"""
        response = self._runcode('globals().keys()');
        if(response is None):
            return;
        return response.result[2:-2].split("', '");
            

    def get__doc__(self, objtxt):
        """Get object __doc__"""
        if(objtxt is None):
            return;
        source = 'help('+objtxt+')';
        response = self._runcode(source);
        if(response is None):
            return;
        return response.standardoutput;

    def __flush_eventqueue(self):
        """Flush keyboard event queue"""
        while self.eventqueue:
            past_event = self.eventqueue.pop(0)
            self.postprocess_keyevent(past_event)

    #------ Keyboard events
    def on_enter(self, command):
        """on_enter"""
        if len(command) > 0:
            self.add_to_history(command)
            # handle multi-line commands
            if command[-1] == self._multi_line_char:
                self._command += self._multi_line_indent * self._multi_line_level + command + '\n'
                self._multi_line = True
                self._multi_line_level += 1
            elif self._multi_line:
                self._command += self._multi_line_indent * self._multi_line_level + command + '\n'
            else:  # single line command
                self.execute_command(command)
                self._command = ''
        else:  # new line was is empty
            if self._multi_line:  # multi line done
              self.execute_command(self._command)
              self._command = ''
              self._multi_line = False
              self._multi_line_level = 0
        self.new_prompt(self._prompt[self._multi_line] + self._multi_line_indent * self._multi_line_level);
        self.__flush_eventqueue()

    def run_command(self, code, history=True):
        if not code:
            code = ''
        self.interpreter.redirect_stds()
        #super(SpyderConsoleWidget, self).run_command("")
        response = self._runcode(code)
        if(response != None):
            if response.standardoutput != "":
                print(response.standardoutput[:-1])
            if response.standarderror != "":
                print(response.standarderror[:-1])
            elif response.result != "None":
                print(response.result)
        self.flush()
        self.interpreter.restore_stds()
        if(self.log is not None):
            self.log.write(code+'\n')
            self.log.flush()

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
                return response;
            except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
                print("Connection to remote server lost. Reconnecting...")
                self._client = rospy.ServiceProxy(
                    'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
                if retry:
                    self.cache = source
                    self._runcode(code, False)
                else:
                    print("Failed to connect. Is Stack of Tasks running?")
        return None;

    def shutdown(self):
        self.exit_interpreter()
