from telnetlib import Telnet
import threading
import sys
import socket
import re
import time
import unittest
import os
import shutil
import jsbsim
import xml.etree.ElementTree as et
import tempfile

__all__ = ['JSBSimServer']


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


class TelnetInterface:
    def __init__(self, fdm, end_time, port):
        # Execute JSBSim in a separate thread
        self.cond = threading.Condition()
        self.thread = JSBSimThread(fdm, self.cond, end_time, time.time())
        self.thread.start()

        self.cond.acquire()
        self.cond.wait()
        try:
            self.tn = Telnet('localhost', port, timeout=2.0)
        finally:
            self.cond.release()

    def __del__(self):
        if 'tn' in self.__dict__.keys():  # Check if the Telnet session has been successfully opened
            self.tn.close()
        self.thread.quit = True
        self.thread.join()
        del self.thread

    def send_command(self, command):
        self.cond.acquire()
        self.tn.write("{}\n".format(command).encode())
        self.cond.wait()
        msg = self.tn.read_very_eager().decode()
        self.cond.release()
        self.thread.join(0.1)
        return msg

    def get_sim_time(self):
        self.cond.acquire()
        self.cond.wait()
        t = self.thread._fdm.get_sim_time()
        self.cond.release()
        return t

    def get_delta_t(self):
        self.cond.acquire()
        self.cond.wait()
        dt = self.thread._fdm.get_delta_t()
        self.cond.release()
        return dt

    def get_property_value(self, property):
        msg = self.send_command('get '+property).split('\n')
        return float(msg[0].split('=')[1])

    def get_output(self):
        self.cond.acquire()
        self.cond.wait()
        out = self.tn.read_very_eager()
        self.cond.release()
        return out

    def wait(self, seconds):
        self.thread.join(seconds)

    def set_real_time(self, rt):
        self.thread.realTime = rt


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
    # _fdm = jsbsim.FGFDMExec(os.path.join(sandbox.path_to_jsbsim_file(), ''), pm)
    print(sandbox.abs_path_to_jsbsim_file())
    _fdm = jsbsim.FGFDMExec(sandbox.abs_path_to_jsbsim_file(), pm)
    path = sandbox.abs_path_to_jsbsim_file()
    _fdm.set_aircraft_path(os.path.join(path, 'aircraft'))
    _fdm.set_engine_path(os.path.join(path, 'engine'))
    _fdm.set_systems_path(os.path.join(path, 'systems'))
    return _fdm


def check_xml_file(f, header):
    # Is f a file ?
    if not os.path.isfile(f):
        return False
    # Is f an XML file ?
    try:
        tree = et.parse(f)
    except et.ParseError:
        return False
    # Check the file header
    return tree.getroot().tag.upper() == header.upper()

def append_xml(name):
    if len(name) < 4 or name[-4:] != '.xml':
        return name+'.xml'
    return name

def copy_aircraft_def(script_path, sandbox):
    # Get the aircraft name
    tree = et.parse(script_path)
    use_element = tree.getroot().find('use')
    aircraft_name = use_element.attrib['aircraft']

    # Then, create a directory aircraft/aircraft_name in the build directory
    aircraft_path = os.path.join('aircraft', aircraft_name)
    path_to_jsbsim_aircraft = sandbox.path_to_jsbsim_file(aircraft_path)
    if not os.path.exists(aircraft_path):
        os.makedirs(aircraft_path)

    # Make a copy of the initialization file in
    # build/.../aircraft/aircraft_name
    IC_file = append_xml(use_element.attrib['initialize'])
    shutil.copy(os.path.join(path_to_jsbsim_aircraft, IC_file), aircraft_path)

    tree = et.parse(os.path.join(path_to_jsbsim_aircraft, aircraft_name+'.xml'))

    # The aircraft definition file may also load some data from external files.
    # If so, we need to copy these files in our directory
    # build/.../aircraft/aircraft_name Only the external files that are in the
    # original directory aircraft/aircraft_name will be copied. The files
    # located in 'engine' and 'systems' do not need to be copied.
    for element in list(tree.getroot()):
        if 'file' in element.keys():
            name = append_xml(element.attrib['file'])
            name_with_path = os.path.join(path_to_jsbsim_aircraft, name)
            subdirs = os.path.split(name)[0]
            if os.path.exists(name_with_path):
                shutil.copy(name_with_path, os.path.join(aircraft_path,
                                                         subdirs))
            else:
                name_with_system_path = os.path.join(path_to_jsbsim_aircraft,
                                                     'Systems', name)

                if os.path.exists(name_with_system_path):
                    system_path = sandbox(aircraft_path, 'Systems')
                    if not os.path.exists(system_path):
                        os.makedirs(system_path)
                    shutil.copy(name_with_system_path,
                                os.path.join(system_path, subdirs))

    return tree, aircraft_name, path_to_jsbsim_aircraft


class JSBSimTestCase(unittest.TestCase):
    def __init__(self, methodName):
        unittest.TestCase.__init__(self, methodName)
        self._fdm = None

    def set_up(self, *args):
        self.sandbox = Sandbox(*args)
        self.currentdir = os.getcwd()
        os.chdir(self.sandbox())

    def tear_down(self):
        self.delete_fdm()
        os.chdir(self.currentdir)
        self.sandbox.erase()

    def script_list(self, blacklist=[]):
        script_path = self.sandbox.path_to_jsbsim_file('C:/Users/quessy/Dev/jsbsim/scripts/')
        for f in os.path.join(script_path, f):
            if f in blacklist:
                continue

            fullpath = os.path.join(script_path, f)

            # does f contain a JSBSim script ?
            if check_xml_file(fullpath, 'runscript'):
                yield fullpath

    def create_fdm(self, pm=None):
        self._fdm = create_fdm(self.sandbox, pm)
        return self._fdm

    def delete_fdm(self):
        if self._fdm:
            del self._fdm
            self._fdm = None

    def load_script(self, script_name):
        script_path = self.sandbox.path_to_jsbsim_file('C:/Users/quessy/Dev/jsbsim/scripts/', append_xml(script_name))
        self._fdm.load_script(script_path)

    def get_aircraft_xml_tree(self, script_name):
        script_path = self.sandbox.path_to_jsbsim_file('C:/Users/quessy/Dev/jsbsim/scripts/', append_xml(script_name))
        tree = et.parse(script_path)
        use_element = tree.getroot().find('use')
        aircraft_name = use_element.attrib['aircraft']

        aircraft_path = self.sandbox.path_to_jsbsim_file('C:/Users/quessy/Dev/jsbsim/aircraft/', aircraft_name,
                                                         aircraft_name+'.xml')
        return et.parse(aircraft_path)


class TestInputSocket(JSBSimTestCase):
    def set_up(self):
        JSBSimTestCase.set_up(self)
        self.script_path = self.sandbox.path_to_jsbsim_file('C:/Users/quessy/Dev/jsbsim/scripts/', 'c1722.xml')

    def sanity_check(self, _tn):
        # Check if a connection has been established
        out = _tn.get_output().decode()
        self.assertTrue(out.split('\n')[0] == 'Connected to JSBSim server',
                        msg = "Not connected to the JSBSim server. \n Got message '%s' instead" % (out,))

        self.assertEqual(sorted(map(lambda x: x.split('{')[0].strip(), _tn.send_command('help').split('\n')[2:-2])),
                         ['get', 'help', 'hold', 'info', 'iterate', 'quit', 'resume', 'set'])

    def test_no_input(self):
        fdm = create_fdm(self.sandbox)
        fdm.load_script(self.script_path)
        fdm.run_ic()
        fdm.hold()

        with self.assertRaises(socket.error):
            TelnetInterface(fdm, 5., 2222)

    def test_input_socket(self):
        TestInputSocket.set_up(self)
        # First, extract the time step from the script file
        tree = et.parse(self.script_path)
        dt = float(tree.getroot().find('run').attrib['dt'])

        # The aircraft c172x does not contain an <input> tag so we need to add one.
        tree, aircraft_name, b = copy_aircraft_def(self.script_path, self.sandbox)
        root = tree.getroot()
        input_tag = et.SubElement(root, 'input')
        input_tag.attrib['port'] = '1137'
        tree.write(self.sandbox('C:/Users/quessy/Dev/jsbsim/aircraft/', aircraft_name, aircraft_name+'.xml'))

        fdm = create_fdm(self.sandbox)
        fdm.set_aircraft_path('C:/Users/quessy/Dev/jsbsim/aircraft/')
        fdm.load_script(self.script_path)
        fdm.run_ic()
        fdm.hold()

        tn = TelnetInterface(fdm, 5., 1137)
        self.sanity_check(tn)

        # Check the aircraft name and its version
        msg = tn.send_command("info").split('\n')
        self.assertEqual(msg[2].split(':')[1].strip(), root.attrib['name'].strip())
        self.assertEqual(msg[1].split(':')[1].strip(), root.attrib['version'].strip())

        # Check that the simulation time is 0.0
        self.assertEqual(float(msg[3].split(':')[1].strip()), 0.0)
        self.assertEqual(tn.get_sim_time(), 0.0)
        self.assertEqual(tn.get_property_value("simulation/sim-time-sec"), 0.0)

        # Check the time step we get through the socket interface
        self.assertEqual(tn.get_delta_t(), dt)

        # Check that 'iterate' iterates the correct number of times
        tn.send_command("iterate 19")
        self.assertEqual(tn.get_sim_time(), 19. * tn.get_delta_t())
        self.assertAlmostEqual(tn.get_property_value("simulation/sim-time-sec"), tn.get_sim_time(), delta=1E-5)

        # Wait a little bit and make sure that the simulation time has not
        # changed meanwhile thus confirming that the simulation is on hold.
        tn.wait(0.1)
        self.assertEqual(tn.get_sim_time(), 19. * tn.get_delta_t())
        self.assertAlmostEqual(tn.get_property_value("simulation/sim-time-sec"), tn.get_sim_time(), delta=1E-5)

        # Modify the tank[0] contents via the "send" command
        half_contents = 0.5 * tn.get_property_value("propulsion/tank/contents-lbs")
        tn.send_command("set propulsion/tank/contents-lbs " + str(half_contents))
        self.assertEqual(tn.get_property_value("propulsion/tank/contents-lbs"), half_contents)

        # Check the resume/hold commands
        tn.set_real_time(True)
        t = tn.get_sim_time()
        tn.send_command("resume")
        tn.wait(0.5)
        self.assertNotEqual(tn.get_sim_time(), t)
        tn.wait(0.5)
        tn.send_command("hold")
        tn.set_real_time(False)
        t = tn.get_sim_time()
        self.assertAlmostEqual(tn.get_property_value("simulation/sim-time-sec"), t, delta=1E-5)

        # Wait a little bit and make sure that the simulation time has not
        # changed meanwhile thus confirming that the simulation is on hold.
        tn.wait(0.1)
        self.assertEqual(tn.get_sim_time(), t)
        self.assertAlmostEqual(tn.get_property_value("simulation/sim-time-sec"), t, delta=1E-5)

    def test_script_input(self):
        tree = et.parse(self.script_path)
        input_tag = et.SubElement(tree.getroot(), 'input')
        input_tag.attrib['port'] = '1138'
        tree.write('c1722_1.xml')

        fdm = create_fdm(self.sandbox)
        fdm.load_script('c1722_1.xml')
        fdm.run_ic()
        fdm.hold()

        tn = TelnetInterface(fdm, 5., 1138)
        self.sanity_check(tn)


def main():
    jsbsim_dir = Sandbox()
    rel_script_path = 'scripts/c1722.xml'
    print(jsbsim_dir.abs_path_to_jsbsim_file())
    abs_script_path = jsbsim_dir.abs_path_to_jsbsim_file() + '/' + rel_script_path
    xml_setup(abs_script_path)
    tree, aircraft_name, path_to_jsbsim_aircraft = get_aircraft_def(abs_script_path, jsbsim_dir)
    root = tree.getroot()
    input_tag = et.SubElement(root, 'input')
    input_tag.attrib["port"] = "1140"
    print('input port is:', input_tag.attrib['port'])
    tree.write(jsbsim_dir(aircraft_name + '.xml'))
    fdm = create_fdm(jsbsim_dir)
    fdm.set_aircraft_path('aircraft')
    fdm.load_script(rel_script_path)
    fdm.run_ic()
    fdm.hold()
    run_telnet(fdm, 5., 1137)
    jsbsim_dir.delete_csv_files()
    jsbsim_dir.erase()

def get_aircraft_def(script_path, sandbox):
    # Get the aircraft name
    tree = et.parse(script_path)
    use_element = tree.getroot().find('use')
    aircraft_name = use_element.attrib['aircraft']

    # Then, create a directory aircraft/aircraft_name in the build directory
    aircraft_path = os.path.join('aircraft', aircraft_name)
    path_to_jsbsim_aircraft = sandbox.abs_path_to_jsbsim_file(aircraft_path)
    if not os.path.exists(aircraft_path):
        os.makedirs(aircraft_path)
        print('aircraft directory not found, making one now')

    IC_file = append_xml(use_element.attrib['initialize'])
    shutil.copy(os.path.join(path_to_jsbsim_aircraft, IC_file), aircraft_path)

    tree = et.parse(os.path.join(path_to_jsbsim_aircraft, aircraft_name + '.xml'))

    print('dest:', os.path.join(aircraft_path, aircraft_name + '.xml'))
    shutil.copy(os.path.join(path_to_jsbsim_aircraft, aircraft_name + '.xml'),
                sandbox(aircraft_name + '.xml'))

    # The aircraft definition file may also load some data from external files.
    # If so, we need to copy these files in our directory
    # build/.../aircraft/aircraft_name Only the external files that are in the
    # original directory aircraft/aircraft_name will be copied. The files
    # located in 'engine' and 'systems' do not need to be copied.
    for element in list(tree.getroot()):
        if 'file' in element.keys():
            name = append_xml(element.attrib['file'])
            name_with_path = os.path.join(path_to_jsbsim_aircraft, name)
            subdirs = os.path.split(name)[0]
            print('subdirs are:', subdirs)
            print(name_with_path)
            if os.path.exists(name_with_path):
                shutil.copy(name_with_path, os.path.join(aircraft_path, subdirs))
            else:
                name_with_system_path = os.path.join(path_to_jsbsim_aircraft,
                                                     'Systems', name)

                if os.path.exists(name_with_system_path):
                    system_path = sandbox(aircraft_path, 'Systems')
                    if not os.path.exists(system_path):
                        os.makedirs(system_path)
                    shutil.copy(name_with_system_path,
                                os.path.join(system_path, subdirs))

    return tree, aircraft_name, path_to_jsbsim_aircraft


def run_telnet(fdm, end_time, port):
    print(fdm)
    print(end_time)
    print(port)
    tn = TelnetInterface(fdm, end_time, port)
    print('sim time is:', tn.get_sim_time())
    print('dt is:', tn.get_delta_t())
    msg = tn.send_command("info")
    print(msg)
    # tn.send_command("iterate 19")
    # print('sim time is:', tn.get_property_value("simulation/sim-time-sec"))


def xml_setup(script_path):
    tree = et.parse(script_path)
    dt = float(tree.getroot().find('run').attrib['dt'])
    root = tree.getroot()
    input_tag = et.SubElement(root, 'input')
    input_tag.attrib['port'] = '1137'





main()
