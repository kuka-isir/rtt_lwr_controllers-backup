#ifndef __FRI_USER_DATA_DESCRIPTION__
#define __FRI_USER_DATA_DESCRIPTION__

namespace krl{

enum FRI_BOOL_DATA_DESCRIPTION
{
    KRL_LOOP_REQUESTED = 0,
    CANCEL_MOTION,
    SET_CONTROL_MODE,
    SET_TOOL,
    SET_BASE,
    PTP_CMD, // send a PTP command
    LIN_CMD, // send a lin command (see LIN_CMD_TYPE)
    STOP2, // Send a stop2 (brakes but still active)
    SET_VEL, // Set the max vel (should modify the value on teach upper right)
    MASK_0, // Mask used for A1-A6 (PTP) or XYZABC (LIN)
    MASK_1,
    MASK_2,
    MASK_3,
    MASK_4,
    MASK_5,
    MASK_6
}FriBoolDataToKRL;

enum FRI_INT_DATA_DESCRIPTION
{
    FRI_STATE = 0,
    CONTROL_MODE,
    TOOL,
    BASE,
    FRI_CMD,
    USE_RELATIVE, // 0 is false, 1 is true
    CMD_INPUT_TYPE // 0 is Joint, 1 is Cartesian in base Frame, 2 is Cartesian in Tool Frame
};

enum COMMAND_INPUT_TYPE
{
    JOINT = 0,
    CARTESIAN_IN_BASE,
    CARTESIAN_IN_TOOL
};

enum FRI_COMMAND
{
    FRI_START = 0,
    FRI_STOP,
    FRI_CLOSE
};

enum FRI_REAL_DATA_DESCRIPTION
{
    X = 0, // x in millimeters
    Y, // y in millimeters
    Z, // z in millimeters
    A, // rotation around Z in degrees
    B, // rotation around Y in degrees
    C, // rotation around X in degrees
    CMD_VEL_PERCENT, // Vel for PTP, lin etc.
    REAL_EMPTY7,
    OV_VEL_PERCENT, // Overrride Max velocity in % (upper right on teach)
    A1, // Joint 0 in degrees
    A2, // Joint 1 in degrees
    E1, // Joint 2 in degrees
    A3, // Joint 3 in degrees
    A4, // Joint 4 in degrees
    A5, // Joint 5 in degrees
    A6  // Joint 6 in degrees
};

enum FRI_REAL_DATA_FROM_KRL_DESCRIPTION
{
    BASE_X = 0, // x in millimeters
    BASE_Y, // y in millimeters
    BASE_Z, // z in millimeters
    BASE_A, // rotation around Z in degrees
    BASE_B, // rotation around Y in degrees
    BASE_C, // rotation around X in degrees
    POS_ACT_X,
    POS_ACT_Y,
    POS_ACT_Z,
    AXIS_ACT_A1, // Joint 0 in degrees
    AXIS_ACT_A2, // Joint 1 in degrees
    AXIS_ACT_E1, // Joint 2 in degrees
    AXIS_ACT_A3, // Joint 3 in degrees
    AXIS_ACT_A4, // Joint 4 in degrees
    AXIS_ACT_A5, // Joint 5 in degrees
    AXIS_ACT_A6  // Joint 6 in degrees
};

}
#endif
