std_msgs/Header header 
float64  valid        # percentage of valid package recived
float64 cycle_dur #the duration of the cycle operation in seconds
float64  write2read_dur        # duration left to the pi3hat to finish the communication
string[] name            # motors id
float64[] pack_loss   # percentage of corrected recived packet identify by motor id