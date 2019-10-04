package frc.team4373.robot;

/**
 * Holds various mappings and constants.
 */

public class RobotMap {
    // Motor inversions
    public static final boolean DRIVETRAIN_MOTOR_RIGHT_1_INVERTED = false;
    public static final boolean DRIVETRAIN_MOTOR_RIGHT_2_INVERTED = false;
    public static final boolean DRIVETRAIN_MOTOR_LEFT_1_INVERTED = true;
    public static final boolean DRIVETRAIN_MOTOR_LEFT_2_INVERTED = true;
    public static final boolean DRIVETRAIN_MOTOR_MIDDLE_1_INVERTED = false;
    public static final boolean DRIVETRAIN_MOTOR_MIDDLE_2_INVERTED = false;
    public static final boolean LIFT_MOTOR_1_INVERTED = true;
    public static final boolean LIFT_MOTOR_2_INVERTED = true;
    public static final boolean CLIMBER_MOTOR_INVERTED = true;
    public static final boolean INTAKE_MOTOR_RIGHT_INVERTED = false;
    public static final boolean INTAKE_MOTOR_LEFT_INVERTED = false;

    // Sensor phase configuration
    public static final boolean DRIVETRAIN_RIGHT_ENCODER_PHASE = false;
    public static final boolean DRIVETRAIN_LEFT_ENCODER_PHASE = false;
    public static final boolean DRIVETRAIN_MIDDLE_SENSOR_PHASE = true;

    // Speed presets
    public static final double LIFT_MOVEMENT_SPEED = 0.75;
    public static final int INTAKE_MOTOR_OUTPUT = 1;
    public static final double AUTON_MIDDLE_WHEEL_ADJUSTMENT_SPEED = 0.5;
    public static final double AUTON_TURN_SPEED = 0.2;
    public static final double AUTON_LONG_DRIVE_SPEED = 1; // for driving long distances
    public static final double AUTON_VISION_APPROACH_SPEED = 0.25;
    public static final double MAXIMUM_CLIMBER_DRIVE_SPEED = 0.333;

    // OI devices
    public static final int DRIVE_JOYSTICK_PORT = 0;
    public static final int OPERATOR_JOYSTICK_PORT = 1;

    // Buttons
    public static final int OPERATOR_TRIGGER_COLLECT_HATCH = 2; // L trigger
    public static final int OPERATOR_TRIGGER_RELEASE_HATCH = 3; // R trigger
    public static final int OPERATOR_BUTTON_COLLECT_CARGO = 5; // left button
    public static final int OPERATOR_BUTTON_RELEASE_CARGO = 6; // right button
    public static final int OPERATOR_BUTTON_LIFT_CARGO_L2 = 2; // b button
    public static final int OPERATOR_BUTTON_LIFT_CARGO_L1 = 1; // a button
    public static final int OPERATOR_BUTTON_LIFT_CARGO_SHIP = 3; // x button
    public static final int OPERATOR_BUTTON_LIFT_TO_LOAD = 8; // start button
    public static final int OPERATOR_BUTTON_LIFT_TO_GROUND = 7; // back button
    public static final int OPERATOR_BUTTON_TOGGLE_INTAKE = 10; // press unused (right) stick
    public static final int OPERATOR_AXIS_LIFT_MANUAL_CONTROL = 1; //Left stick Y, up is negative.
    // public static final int DRIVER_BUTTON_CLIMB_RAISE_BOT = 7;
    public static final int DRIVER_BUTTON_CLIMB_RAISE_BOT_FRONT = 7;
    public static final int DRIVER_BUTTON_CLIMB_RAISE_BOT_REAR = 8;
    public static final int DRIVER_BUTTON_CLIMB_RETRACT_FRONT = 9;
    public static final int DRIVER_BUTTON_CLIMB_RETRACT_REAR = 10;
    public static final int DRIVER_BUTTON_CLIMB_DRIVE_BACKWARD = 11; // n.b. no longer 12
    public static final int DRIVER_BUTTON_STRAFE_ONLY = 11; // if climb, button 3
    public static final int DRIVER_BUTTON_DRIVE_STRAIGHT = 12; // if climb, button 5
    public static final int DRIVER_BUTTON_VISION_ALIGNMENT = 2;
    public static final int DRIVER_BUTTON_KILL_AUTON = 6;
    public static final int DRIVER_BUTTON_TOGGLE_LIGHT_RING = 12;
    public static final int DRIVER_BUTTON_BASIC_DRIVE = 4;

    public static final int DRIVER_AXIS_SLIDER_CLIMBER_WHEEL = 3;

    // Motor CAN chain identifiers
    public static final int DRIVETRAIN_MOTOR_RIGHT_1 = 13;
    public static final int DRIVETRAIN_MOTOR_RIGHT_2 = 14;
    public static final int DRIVETRAIN_MOTOR_LEFT_1 = 11;
    public static final int DRIVETRAIN_MOTOR_LEFT_2 = 12;
    public static final int DRIVETRAIN_MOTOR_MIDDLE_1 = 15;
    public static final int DRIVETRAIN_MOTOR_MIDDLE_2 = 16;
    public static final int LIFT_MOTOR_1 = 21;
    public static final int LIFT_MOTOR_2 = 22;
    public static final int CLIMBER_DRIVE_MOTOR = 31;
    public static final int INTAKE_MOTOR_RIGHT = 41;
    public static final int INTAKE_MOTOR_LEFT = 42;

    // Pneumatic channel mappings
    public static final int DRIVETRAIN_PISTON_FORWARD = 0; // pcm 1
    public static final int DRIVETRAIN_PISTON_BACKWARD = 1; // pcm 1
    public static final int INTAKE_PISTON_DEPLOYMENT_FORWARD = 2; // pcm 1
    public static final int INTAKE_PISTON_DEPLOYMENT_BACKWARD = 3; // pcm 1
    public static final int CLIMBER_PISTON_REAR_FORWARD = 4; // pcm 1
    public static final int CLIMBER_PISTON_REAR_BACKWARD = 5; // pcm 1
    public static final int CLIMBER_PISTON_FRONT_FORWARD = 6; // pcm 1
    public static final int CLIMBER_PISTON_FRONT_BACKWARD = 7; // pcm 1
    public static final int LIFT_PISTON_BACKWARD = 1; // pcm 2
    public static final int LIFT_PISTON_FORWARD = 7; // pcm 2
    public static final int INTAKE_PISTON_HATCH_BACKWARD = 0; // pcm 2
    public static final int INTAKE_PISTON_HATCH_FORWARD = 2; // pcm 2

    // Digital input mappings
    public static final int INTAKE_LIMIT_SWITCH_CHANNEL = 2;
    public static final int LIGHT_RING_RELAY_CHANNEL = 3;

    // Other mappings
    public static final int PTN_CHANNEL = 1;

    // Control system ports
    public static final int PCM_1_PORT = 2;
    public static final int PCM_2_PORT = 3;

    // PID- and sensor-related constants
    public static class PID {
        public double kP;
        public double kI;
        public double kD;

        PID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    // 6in diameter wheels; 10.71:1 gearbox ratio; 4096 encoder units per rotation
    public static final double DRIVETRAIN_SIDE_ENC_UNITS_TO_IN = 6 * Math.PI / 4096 / 10.71;
    public static final double DRIVETRAIN_MID_ENC_UNITS_TO_IN = 6 * Math.PI / 4096;
    // converts from units/0.1s to in/s
    public static final double DRIVETRAIN_ENC_VEL_TO_INPS = 6 * 10 * Math.PI / 4096 / 10.71;
    public static final double LIFT_PTN_TO_ARM_CHAIN_RATIO = 10d / 3d; // 10/3 ptn turns = 1 motor
    public static final int LIFT_MAX_POTEN_VALUE = 1000;
    public static final int LIFT_INITIAL_ANG_OFFSET = 0;
    public static final double LIFT_ARM_LENGTH = 39;
    public static final double LIFT_ARM_MOUNT_HEIGHT = 39; // from floor, not bottom of bot
    public static final double LIFT_MAXIMUM_SAFE_ANGLE_1 = 380;
    public static final double LIFT_MINIMUM_SAFE_ANGLE_1 = 12;
    public static final double LIFT_MAXIMUM_SAFE_ANGLE_2 = 580;
    public static final double LIFT_MINIMUM_SAFE_ANGLE_2 = 250;
    // allowable percent output increase on lift between cycles
    public static final double LIFT_MAXIMUM_RAMP_INCREASE = 0.02;

    // PID gains
    public static final PID LIFT_PID_GAINS = new PID(0.009, 0, 0.025);
    public static final PID DRIVETRAIN_DIST_PID_GAINS = new PID(0.0175, 0, 0.03);
    public static final PID DRIVETRAIN_ANG_PID_GAINS = new PID(0.009, 0, 0.008); // tuned@0.2speed
    public static final PID DRIVETRAIN_MIDDLE_PID_GAINS = new PID(0.0025, 0, 0.0025);

    // Vision
    public static final double VISION_SAMPLE_COUNT = 10;
    public static final double ALLOWABLE_LATERAL_OFFSET_FROM_VIS_TARGET = 1.5; // inches
    public static final double ALLOWABLE_ANGLE_TO_VIS_TARGET = 3; // degrees
    public static final double ALLOWABLE_ERR_DISTANCE_TO_VIS_TARGET = 4; // inches

    public enum Side {
        RIGHT, LEFT, MIDDLE
    }

    public enum CargoShipPort {
        NEAR, MIDDLE, FAR
    }

    public enum RocketHatchPanel {
        NEAR, FAR
    }

    public enum RocketHeight {
        LOW, MIDDLE
    }

}
