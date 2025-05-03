# Colors from console print
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"
BOLD = "\033[1m"
UNDERLINE = "\033[4m"
REVERSED = "\033[7m"

# Home positions
SGDB_LAT = 48.6126523
SGDB_LON = 2.3963258
DUAV_LAT = 45.4389466
DUAV_LON = -0.4283327
DEFAULT_LAT = SGDB_LAT
DEFAULT_LON = SGDB_LON
DEFAULT_ALT = 6

# Global parameters
PI = 3.141592653589
EARTH_RAD = 6371000
TO_RAD = 180/PI
TO_DEG = PI/180
CONV = TO_RAD/EARTH_RAD

# Debug
on_raspi = False
debug_cams = True
if on_raspi:
    debug_cams = False

# Path
path_step = "/home/step/ros2_DUAV"
path_felix = "/home/linux/ros2_DUAV"
path_popeye = "/home/duav/ros2_DUAV"
path_DUAV = path_step

############################################################################################################################################################################################################################
##### CONVERSION UTILS ############################################################################################################################################################################################################################
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to filter if the uav is not flat ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def uav_is_tilted(attitude_roll, attitude_pitch):
    roll_angle = attitude_roll*TO_RAD
    pitch_angle = attitude_pitch*TO_RAD
    if (roll_angle>5 or roll_angle<-5) or (pitch_angle>5 or pitch_angle<-5):
        return True
    return False
