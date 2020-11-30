import airsim
import jsbsim

# Connect to airsim client and get pose data
client = airsim.VehicleClient()
client.confirmConnection()
pose = client.simGetVehiclePose()





