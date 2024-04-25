string display_mode
string last_command #MISSION or DIRECT
float64 timestamp
bool is_recording
string capture_mode

uint16 zoom                 # range: 10 <= x <= 1000

# float32 shutter_speed_numerator		# range: 1 <= x <= 300
uint16 shutter_speed_numerator		# range: 1 <= x <= 300
uint16 shutter_speed_denominator    # range: 1 <= x <= 8000
float32 f_stop				# range: 0.5 <= x
uint64 iso					# range: 100 <= x
bool    iso_is_auto 

float32 exposure_bias		# default: -5 <= x <= 5
uint16 	white_balance		# default: 1000 <= x <= 10000
# string  white_balance_name
bool    white_balance_is_auto 

string exposure_mode        # P, A, S, M

bool default_auto_focus
int16 focus_point_x
int16 focus_point_y
string focus_size           # SMALL, MEDIUM, LARGE, WIDE

int16 thermal_range_maximum
int16 thermal_range_minimum
string thermal_mode			# AUTO, MANUAL, SPAN
string thermal_palette		# GREY

string camera_type          # RGB, IR, RGB/IR
string camera