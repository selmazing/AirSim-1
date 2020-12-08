from telnetlib import Telnet
import jsbsim
import os
import shutil
import tempfile
import sys
import threading
import time


class JSBSimThread(threading.Thread):
    def __init__(self, fdm, cond, end_time, t0=0.0):
        threading.Thread.__init__(self)
        self.realTime = False
        self.quit = False
        self._fdm = fdm
        self._cond = cond
        self._end_time = end_time
        self._t0 = t0

    def __del__(self):
        del self._fdm

    def run(self):
        self._cond.acquire()
        current_sim_time = self._fdm.get_sim_time()
        self._cond.release()

        while not self.quit:
            if current_sim_time > self._end_time:
                return
            if not self.realTime or current_sim_time < (time.time()-self._t0):
                self._cond.acquire()
                if not self._fdm.run():
                    self._cond.release()
                    return
                self._fdm.check_incremental_hold()
                current_sim_time = self._fdm.get_sim_time()
                self._cond.notify()
                self._cond.release()


class TelnetInterface(Telnet):
    def __init__(self, fdm, host, end_time, port):
        Telnet.__init__(self, host, port)
        self.cond = threading.Condition()
        self.thread = JSBSimThread(fdm, self.cond, end_time, time.time())
        self.thread.start()
        self.end_time = end_time

        self.cond.acquire()
        self.cond.wait()
        try:
            self.tn = Telnet('localhost', port, timeout=2.0)
            self.tn.set_debuglevel(1)
        finally:
            self.cond.release()
            print('Releasing thread!!!')

    def __del__(self):
        if 'tn' in self.__dict__.keys():  # Check if the Telnet session has been successfully opened
            self.tn.close()
        print('Connection is being closed!!!')
        self.thread.quit = True
        self.thread.join()
        del self.thread

    def get_sim_time(self):
        self.cond.acquire()
        self.cond.wait()
        t = self.thread._fdm.get_sim_time()
        self.cond.release()
        return t

    def pwd(self):
        """Display current path."""
        self._putcmd('pwd')
        return self._getresp()

    def send_command(self, command):
        self.cond.acquire()
        self.tn.write("{}\n".format(command).encode())
        self.cond.wait()
        msg = self.tn.read_very_eager().decode()
        self.cond.release()
        self.thread.join(0.1)
        return msg

    def get_property_value(self, property):
        msg = self.send_command('get '+property).split('\n')
        print(msg)
        return msg

    # send one command to JSBSim
    def _putcmd(self, cmd):
        cmd = cmd
        self.tn.write("{}\n".format(cmd).encode())
        return

    # get a response from JSBSim
    def _getresp(self):
        self.cond.acquire()
        self.cond.wait()
        out = self.tn.read_very_eager()
        self.cond.release()
        print(out)
        return out


class Sandbox:
    def __init__(self, *args):
        self._tmpdir = tempfile.mkdtemp(dir=os.getcwd())
        self.path_to_jsbsim = os.path.join(os.path.dirname(sys.argv[0]), 'C:/Users/quessy/Dev/jsbsim', *args)
        self._relpath_to_jsbsim = os.path.relpath(self.path_to_jsbsim, self._tmpdir)

    def __call__(self, *args):
        return os.path.relpath(os.path.join(self._tmpdir, *args), os.getcwd())

    def delete_csv_files(self):
        files = os.listdir(self._tmpdir)
        for f in files:
            if f[-4:] == '.csv':
                os.remove(os.path.join(self._tmpdir, f))

    def abs_path_to_jsbsim_file(self, *args):
        return os.path.join(self.path_to_jsbsim, *args)

    def path_to_jsbsim_file(self, *args):
        return os.path.join(self._relpath_to_jsbsim, *args)

    def exists(self, filename):
        return os.path.exists(self(filename))

    def erase(self):
        shutil.rmtree(self._tmpdir)


def create_fdm(sandbox, pm=None):
    _fdm = jsbsim.FGFDMExec(sandbox.abs_path_to_jsbsim_file(), pm)
    path = sandbox.abs_path_to_jsbsim_file()
    _fdm.set_aircraft_path(os.path.join(path, 'aircraft'))
    _fdm.set_engine_path(os.path.join(path, 'engine'))
    _fdm.set_systems_path(os.path.join(path, 'systems'))
    return _fdm


def initialize_fdm(fdm, script_path):
    fdm.set_aircraft_path('aircraft')
    fdm.load_script(script_path)
    fdm.run_ic()
    fdm.hold()


def main():
    jsb_box = Sandbox()
    script_path = 'scripts/c1722.xml'
    fdm = create_fdm(jsb_box)
    initialize_fdm(fdm, script_path)
    print('Sim time is:', fdm.get_sim_time())

    # JSBSimServerOut = TelnetInterface(fdm, 5., 1138)
    jsb_server = TelnetInterface(fdm, 'localhost',  5., 1137)
    print('Sim time is:', jsb_server.get_sim_time())
    jsb_server.get_property_value("inertia/cg-x-ft")



main()
