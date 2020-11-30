from JSBSim import JSBSimServer
import jsbsim
import time


def main():
    fdm = jsbsim.FGFDMExec('C:/Users/quessy/Dev/jsbsim', None)
    fdm.load_script('scripts/AirSim_Test.xml')
    fdm.get_('data_output/socket.xml')
    fm = JSBSimServer.JSBSimServer('localhost', 1138)
    while 1:
        if fm['/sim/time/elapsed-sec'] > 5:
            break
        time.sleep(1.0)
        print(fm['/sim/time/elapsed-sec'])

    heading = fm['/orientation/heading-deg']
    time.sleep(2.0)
    # fdm.run_ic()

    fm.quit()


if __name__ == '__main__':
    main()
