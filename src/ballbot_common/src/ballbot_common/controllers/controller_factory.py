from ballbot_common.controllers.CADRL import CADRL
from ballbot_common.controllers.MPC import MPC
from ballbot_common.controllers.ORCA import ORCA
from ballbot_common.controllers.PID import PID


controller_factory = dict()
controller_factory['CADRL'] = CADRL
controller_factory['MPC'] = MPC
controller_factory['ORCA'] = ORCA
controller_factory['PID'] = PID
