import tkinter.tix as tix
from tkinter.constants import *
from JSBSim import JSBSimServer
import sys
import socket


class PropertyField:
    def __init__(self, parent, prop, label):
        self.prop = prop
        self.field = tix.LabelEntry(parent, label = label,
                                    options = '''
                                    label.width 30
                                    label.anchor e
                                    entry.width 30''')
        self.field.pack(side=tix.TOP, padx=20, pady=2)

    def update_field(self, fdm):
        val = fdm[self.prop]
        self.field.entry.delete(0, 'end')
        self.field.entry.insert(0, val)


class PropertyPage(tix.Frame):
    def __init__(self, parent, fdm):
        tix.Frame.__init__(self, parent)
        self.fdm = fdm
        self.pack(side=tix.TOP, padx=2, pady=2, fill=tix.BOTH, expand=1)
        self.fields = []

    def addField(self, prop, label):
        f = PropertyField(self, prop, label)
        self.fields.append(f)

    def update_fields(self):
        for f in self.fields:
            f.update_field(self.fdm)
            tix.Frame.update(self)


class JSBSimDemo(tix.Frame):
    def __init__(self, fdm, root = None):
        tix.Frame.__init__(self, root)
        z = root.winfo_toplevel()
        z.wm_protocol("WM_DELETE_WINDOW", lambda self=self: self.quitcmd())
        self.fdm = fdm
        self.pack()
        self.pages = {}
        self.after_id = None
        self.createWidgets()
        self.update()

    def createWidgets(self):
        self.nb = tix.NoteBook(self)
        self.nb.add('sim', label='Simulation', raisecmd=lambda self=self: self.update_page())
        self.nb.add('loc', label='Location', raisecmd=lambda self=self: self.update_page())
        self.nb.add('env', label='Environment', raisecmd=lambda self=self: self.update_page())
        self.nb.add('vel', label='Velocities', raisecmd=lambda self=self: self.update_page())

        page = PropertyPage(self.nb.sim, self.fdm)
        self.pages['sim'] = page
        page.addField('/sim/aircraft', 'Aircraft:')
        page.addField('/sim/time/gmt', 'Current time (GMT):')

        page = PropertyPage(self.nb.view, self.fdm)
        self.pages['loc'] = page
        page.addField('/position/altitude-ft', 'Altitude (ft):')
        page.addField('/position/longitude-deg', 'Longitude (deg):')
        page.addField('/position/latitude-deg', 'Latitude (deg):')
        page.addField('/orientation/roll-deg', 'Roll (deg):')
        page.addField('/orientation/pitch-deg', 'Pitch (deg):')
        page.addField('/orientation/heading-deg', 'Heading (deg):')

        page = PropertyPage(self.nb.env, self.fdm)
        self.pages['env'] = page
        page.addField('/environment/wind-from-heading-deg', 'Wind direction (deg FROM):')
        page.addField('/environment/params/base-wind-speed-kt', 'Wind speed (kt):')
        page.addField('/environment/params/gust-wind-speed-kt', 'Maximum gust (kt):')
        page.addField('/environment/wind-from-down-fps', 'Updraft (fps):')
        page.addField('/environment/temperature-degc', 'Temperature (degC):')
        page.addField('/environment/dewpoint-degc', 'Dewpoint (degC):')
        page.addField('/environment/pressure-sea-level-inhg', 'Altimeter setting (inHG):') # is this in millibars too

        page = PropertyPage(self.nb.env, self.fdm)
        self.pages['vel'] = page
        page.addField('/velocities/airspeed-kt', 'Airspeed (kt):')
        page.addField('/velocities/speed-down-fps', 'Descent speed (fps):')

        self.nb.pack(expand=1, fill=tix.BOTH, padx=5, pady=5, side=tix.TOP)

        self.QUIT = tix.Button(self)
        self.QUIT['text'] = 'Quit'
        self.QUIT['command'] = self.quitcmd
        self.QUIT.pack({'side': 'bottom'})

    def quitcmd(self):
        if self.after_id:
            self.after_cancel(self.after_id)
        self.destroy()

    def update_page(self):
        page = self.pages[self.nb.raised()]
        page.update_field()
        self.update()
        self.after_id = self.after(1000, lambda self = self: self.update_page())


def main():
    if len(sys.argv) != 3:
        print('Usage: %s host port' % sys.argv[0])
        sys.exit(1)

    host = sys.argv[1]
    try:
        port = int(sys.argv[2])
    except ValueError as msg:
        print('Error: expected a number for port')
        sys.exit(1)

    fdm = None
    try:
        port = int(sys.argv[2])
    except ValueError as msg:
        print('Error: expected a number for port')
        sys.exit(1)

    fdm = None
    try:
        fdm = JSBSimServer(host, port)
    except socket.error as msg:
        print('Error: expected a number for port')
        sys.exit(1)

    fdm = None
    try:
        fdm = JSBSimServer(host, port)
    except socket.error as msg:
        print('Error connecting to JSBSim', msg[1])
        sys.exit(1)

    root = tix.Tk()
    app = JSBSimDemo(fdm, root)
    app.mainloop()


if __name__ == '__main__':
    main()
