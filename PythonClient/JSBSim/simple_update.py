import airsim
import jsbsim

client = airsim.VehicleClient()
client.confirmConnection()

pose = client.simGetVehiclePose()
print("x={}, y={}, z={}".format(pose.position.x_val, pose.position.y_val, pose.position.z_val))

fdm = jsbsim.FGFDMExec('C:/Users/quessy/Dev/jsbsim')
fdm.load_script('scripts/f16_test.xml')
fdm.run_ic()

x0 = fdm.get_property_value('position/ecef-x-ft')
y0 = fdm.get_property_value('position/ecef-y-ft')
z0 = fdm.get_property_value('position/ecef-z-ft')

while fdm.run():
    pose.position.x_val = (fdm.get_property_value('position/ecef-x-ft') - x0) / 3.28
    pose.position.y_val = (fdm.get_property_value('position/ecef-y-ft') - y0) / 3.28
    pose.position.z_val = (fdm.get_property_value('position/ecef-z-ft') - z0) / 3.28 - 10
    # print(pose.position)
    roll = fdm.get_property_value('attitude/roll-rad')
    pitch = fdm.get_property_value('attitude/pitch-rad')
    yaw = fdm.get_property_value('attitude/yaw-rad')
    pose.orientation = airsim.to_quaternion(pitch, roll, yaw)
    # print(pose.orientation)
    client.simSetVehiclePose(pose, True)
    # print(x)
    # print(y)
    # print(z)
    # print(fdm.query_property_catalog('orientation'))
