import numpy

#   FUNCTION USED TO TRANSFORM DEGREE IN CIRCUMFERENCE
def deg2circ(deg):
    circ = deg/360
    return circ

#   FUNCIOTN USED TO TRANSFORM CIRCUMFERENCE IN DEGREE
def circ2deg(circ):
    deg = circ*360
    return deg

#   CLASS USED TO INITIALIZE THE MOTOR VARIABLES
class MotorState:
    # servos_list = numpy.array([1, 2, 3, 4, 5, 6, 7, 8])
    # # tank_list = numpy.array([1, 2, 3, 4, 5, 6, 7, 8])
    # hip_list_front = numpy.array([1, 3])
    # tank_list_front = numpy.array([2, 4])    
    # hip_list_hind = numpy.array([5, 7])
    # tank_list_hind = numpy.array([6, 8])
    # servos_list = numpy.array([1, 2, 5, 6])
    servos_list = numpy.array([1])
    hip_list_front_right = numpy.array([3])
    tank_list_front_right = numpy.array([4])  
    hip_list_hind_right = numpy.array([7])
    tank_list_hind_right = numpy.array([8])
    hip_list_front_left = numpy.array([1])
    tank_list_front_left = numpy.array([2])  
    hip_list_hind_left = numpy.array([5])
    tank_list_hind_left = numpy.array([6]) 
    # hip_list_hind = numpy.array([5, 7])
    # tank_list_hind = numpy.array([6, 8])


#   ACTUATOR PARAMETERS
class MActPar:
    MAXVEL = 20
    MAXPOW = 450
<<<<<<< HEAD
    MAXCUR = 40 
    KP = 5.0
    KD = 0.05
=======
    MAXCUR = 5 
    KP = 0.0
    KD = 0.00
>>>>>>> d274abb (fix bug on omnicontroller)
    KI = 0
    FBV = 27.5
    RID = 9


    
# STAND UP PARAMETERS
class StandUpParam:
    TF = 5

    initial_pos_front_hip_right = 0
    final_pos_front_hip_right = 80
    initial_pos_hind_hip_right = 0
    final_pos_hind_hip_right = -80 
    initial_pos_front_tank_right = 0
    final_pos_front_tank_right = -0.27
    initial_pos_hind_tank_right = 0
    final_pos_hind_tank_right = 0.27

    initial_pos_front_hip_left = 0
    final_pos_front_hip_left = -80
    initial_pos_hind_hip_left = 0
    final_pos_hind_hip_left = 80 
    initial_pos_front_tank_left = 0
    final_pos_front_tank_left = -0.27
    initial_pos_hind_tank_left = 0
    final_pos_hind_tank_left = 0.27

class SteadyParam:
    TF = 60

    initial_pos_front_hip_right =StandUpParam.final_pos_front_hip_right
    final_pos_front_hip_right = StandUpParam.final_pos_front_hip_right
    initial_pos_hind_hip_right = StandUpParam.final_pos_hind_hip_right
    final_pos_hind_hip_right = StandUpParam.final_pos_hind_hip_right 
    initial_pos_front_tank_right = StandUpParam.final_pos_front_tank_right
    final_pos_front_tank_right = StandUpParam.final_pos_front_tank_right
    initial_pos_hind_tank_right = StandUpParam.final_pos_hind_tank_right
    final_pos_hind_tank_right = StandUpParam.final_pos_hind_tank_right

    initial_pos_front_hip_left = StandUpParam.final_pos_front_hip_left
    final_pos_front_hip_left = StandUpParam.final_pos_front_hip_left
    initial_pos_hind_hip_left = StandUpParam.final_pos_hind_hip_left
    final_pos_hind_hip_left = StandUpParam.final_pos_hind_hip_left
    initial_pos_front_tank_left = StandUpParam.final_pos_front_tank_left
    final_pos_front_tank_left = StandUpParam.final_pos_front_tank_left
    initial_pos_hind_tank_left = StandUpParam.final_pos_hind_tank_left
    final_pos_hind_tank_left = StandUpParam.final_pos_hind_tank_left

# CLIMBING PARAMETERS
class ClimbingParam:
    TF = 16

    initial_pos_front_hip_right = StandUpParam.final_pos_front_hip_right
    final_pos_front_hip_right = StandUpParam.final_pos_front_hip_right
    initial_pos_hind_hip_right = StandUpParam.final_pos_hind_hip_right
    final_pos_hind_hip_right = StandUpParam.final_pos_hind_hip_right   
    initial_pos_front_tank_right = StandUpParam.final_pos_front_tank_right
    final_pos_front_tank_right = StandUpParam.final_pos_front_tank_right - 1
    initial_pos_hind_tank_right = StandUpParam.final_pos_hind_tank_right
    final_pos_hind_tank_right = StandUpParam.final_pos_hind_tank_right - 1

    initial_pos_front_hip_left = StandUpParam.final_pos_front_hip_left
    final_pos_front_hip_left = StandUpParam.final_pos_front_hip_left
    initial_pos_hind_hip_left = StandUpParam.final_pos_hind_hip_left
    final_pos_hind_hip_left = StandUpParam.final_pos_hind_hip_left   
    initial_pos_front_tank_left = StandUpParam.final_pos_front_tank_left
    final_pos_front_tank_left = StandUpParam.final_pos_front_tank_left + 1
    initial_pos_hind_tank_left = StandUpParam.final_pos_hind_tank_left
    final_pos_hind_tank_left = StandUpParam.final_pos_hind_tank_left + 1

# DESCENDING PARAMETERS
class DescendingParam:
    TF = 60

    initial_pos_front_hip_right = StandUpParam.final_pos_front_hip_right
    final_pos_front_hip_right = StandUpParam.final_pos_front_hip_right
    initial_pos_hind_hip_right = StandUpParam.final_pos_hind_hip_right
    final_pos_hind_hip_right = StandUpParam.final_pos_hind_hip_right
    initial_pos_front_tank_right = ClimbingParam.final_pos_front_tank_right
    final_pos_front_tank_right = ClimbingParam.final_pos_front_tank_right - 2
    initial_pos_hind_tank_right = ClimbingParam.final_pos_hind_tank_right
    final_pos_hind_tank_right = ClimbingParam.final_pos_hind_tank_right - 2

    initial_pos_front_hip_left = StandUpParam.final_pos_front_hip_left
    final_pos_front_hip_left = StandUpParam.final_pos_front_hip_left
    initial_pos_hind_hip_left = StandUpParam.final_pos_hind_hip_left
    final_pos_hind_hip_left = StandUpParam.final_pos_hind_hip_left
    initial_pos_front_tank_left = ClimbingParam.final_pos_front_tank_left +2
    final_pos_front_tank_left = ClimbingParam.final_pos_front_tank_left
    initial_pos_hind_tank_left = ClimbingParam.final_pos_hind_tank_left
    final_pos_hind_tank_left = ClimbingParam.final_pos_hind_tank_left +2

# LIE DOWN PARAMETERS
class LieDownParam:
    TF = 10

    initial_pos_front_hip_right = StandUpParam.final_pos_front_hip_right
    final_pos_front_hip_right = StandUpParam.initial_pos_front_hip_right
    initial_pos_hind_hip_right = StandUpParam.final_pos_hind_hip_right
    final_pos_hind_hip_right = StandUpParam.initial_pos_hind_hip_right
    initial_pos_front_tank_right = DescendingParam.final_pos_front_tank_right
    final_pos_front_tank_right = StandUpParam.initial_pos_front_tank_right
    initial_pos_hind_tank_right = DescendingParam.final_pos_hind_tank_right
    final_pos_hind_tank_right = StandUpParam.initial_pos_hind_tank_right

    initial_pos_front_hip_left = StandUpParam.final_pos_front_hip_left
    final_pos_front_hip_left = StandUpParam.initial_pos_front_hip_left
    initial_pos_hind_hip_left = StandUpParam.final_pos_hind_hip_left
    final_pos_hind_hip_left = StandUpParam.initial_pos_hind_hip_left
    initial_pos_front_tank_left = DescendingParam.final_pos_front_tank_left
    final_pos_front_tank_left = StandUpParam.initial_pos_front_tank_left
    initial_pos_hind_tank_left = DescendingParam.final_pos_hind_tank_left
    final_pos_hind_tank_left = StandUpParam.initial_pos_hind_tank_left 

class SteeringParam:
    v_left = 1
    v_right = -1    

#   TEST PARAMETERS
class TaskParam:
    test_duration = 30
    TF1 = 30.0
    TF2 = TF1
    InitialPos = 0 #express in deg
    FinalPos = 45 #express in deg
    inital_position = 0 #express in m
    final_position = 0.5 #express in m
    FLAG = True
    Offset = 0.0
    TANKBELTLENGTH = 0.351
    CICLESNUMBER = 50
    Counter = 1

#   TRAJECTORY GENERATION
class ThirdDegreeInterpolation:
    interpolation = 0
    interpolation_dot = 0
    interpolation_tank = 0
    interpolation_tank_dot = 0

    def TrajectoryTank(t, TF, InitialPos, FinalPos):    
        a0 = InitialPos
        a1 = 0
        a2 = 3*(FinalPos - InitialPos)/(TF**2)
        a3 = -2*(FinalPos - InitialPos)/(TF**3)
        ThirdDegreeInterpolation.interpolation_tank = ((a3*t**3) + (a2*t**2) + (a1*t) + a0)
        ThirdDegreeInterpolation.interpolation_tank = MActPar.RID*ThirdDegreeInterpolation.interpolation_tank/((TaskParam.TANKBELTLENGTH*15)/31)
        return ThirdDegreeInterpolation.interpolation_tank

    def TrajectoryTank_dot(t, TF, InitialPos, FinalPos):
        a1 = 0
        a2 = 3*(FinalPos - InitialPos)/(TF**2)
        a3 = -2*(FinalPos - InitialPos)/(TF**3)
        ThirdDegreeInterpolation.interpolation_tank_dot = (3*(a3*(t**2)) + 2*(a2*t) + (a1))
        ThirdDegreeInterpolation.interpolation_tank_dot = MActPar.RID*(ThirdDegreeInterpolation.interpolation_tank_dot)/((TaskParam.TANKBELTLENGTH*15)/31)
        return ThirdDegreeInterpolation.interpolation_tank_dot

    def trajectory(t, tf, initial_position, final_position):    
        a0 = initial_position
        a1 = 0
        a2 = 3*(final_position - initial_position)/(tf**2)
        a3 = -2*(final_position - initial_position)/(tf**3)

        ThirdDegreeInterpolation.interpolation = MActPar.RID*((a3*t**3) + (a2*t**2) + (a1*t) + a0)
        ThirdDegreeInterpolation.interpolation = deg2circ(ThirdDegreeInterpolation.interpolation)
        return ThirdDegreeInterpolation.interpolation

    def trajectory_dot(t, tf, initial_position, final_position):        
        a1 = 0
        a2 = 3*(final_position - initial_position)/(tf**2)
        a3 = -2*(final_position - initial_position)/(tf**3)

        ThirdDegreeInterpolation.interpolation_dot = MActPar.RID*((3*a3*t**2) + (2*a2*t) + a1)
        ThirdDegreeInterpolation.interpolation_dot = deg2circ(ThirdDegreeInterpolation.interpolation_dot)

        return ThirdDegreeInterpolation.interpolation_dot