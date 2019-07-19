package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team4373.robot.Robot;
import frc.team4373.robot.commands.teleop.SwerveDriveWithJoystick;

/**
 * A programmatic representation of the robot's drivetrain.
 */
public class Drivetrain extends Subsystem {
    private static volatile Drivetrain instance;

    /**
     * The getter for the Drivetrain class.
     * @return the singleton Drivetrain object.
     */
    public static Drivetrain getInstance() {
        if (instance == null) {
            synchronized (Drivetrain.class) {
                if (instance == null) {
                    instance = new Drivetrain();
                }
            }
        }
        return instance;
    }

    public enum MotorID {
        RIGHT_1, RIGHT_2, LEFT_1, LEFT_2
    }

    private SwerveMotor right1;
    private SwerveMotor right2;
    private SwerveMotor left1;
    private SwerveMotor left2;
    private PigeonIMU pigeon;
    private double initialAngle;

    private Drivetrain() {
        this.right1 = new SwerveMotor(MotorID.RIGHT_1);
        this.right2 = new SwerveMotor(MotorID.RIGHT_2);
        this.left1 = new SwerveMotor(MotorID.LEFT_1);
        this.left2 = new SwerveMotor(MotorID.LEFT_2);

        // this.pigeon = new PigeonIMU(this.right2.getRotatorMotor());
        //
        // this.right2.follow(right1);
        // this.left2.follow(left1);
        //
        // this.right1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // this.right1.setSensorPhase(RobotMap.DRIVETRAIN_RIGHT_ENCODER_PHASE);
        // this.left1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // this.left1.setSensorPhase(RobotMap.DRIVETRAIN_LEFT_ENCODER_PHASE);

        this.initialAngle = getPigeonYaw();
    }

    /**
     * Returns the angle from the given inputs, relative to the field, where 0 is forward and clockwise counts up.
     */
    public static double calculateAngle(double x, double y) {
        return (((360 - (Math.toDegrees(Math.atan(y/x) + (x >= 0 ? 0 : Math.PI)) - 90)) % 360) + 360) % 360;
        //FIXME: This might be Math.atan2(double y, double x) // plus toDegrees // plus some more stuff
        /*
        double a = Math.atan(y/x);

        if (y >= 0) {
            if (x >= 0) {
                return a;
            } else {
                return a + Math.PI;
            }
        } else {
            if (x >= 0) {
                return a;
            } else {
                return a + Math.PI;
            }
        }
        */
    }

    /**
     * Returns the current angle, assuming the initial position was 0.
     */
    public double getAngle() {
        return (((getPigeonYaw()- initialAngle) % 360) + 360) % 360;
    }

    public double getWheelAngle() {
        return 0;//TODO: Wheel angle
    }

    /**
     * Rotates the given motor at the specified power. 1 is clockwise (viewed from the top), -1 is counterclockwise.
     * @param motor
     * @param power
     */
    public void rotate(MotorID motor, double power) {
        //rotate the given motor.
    }

    public void rotate(double power) {
        for (MotorID t: MotorID.values()) {
            rotate(t, power);
        }
    }

    /**
     * Sets safety-checked percent output to the specified motor.
     *
     * <p>Positive values to the middle wheel go to the right.
     *
     * @param motor the motor whose output to set.
     * @param power the percent output on [-1, 1] to set.
     */
    public void setPercentOutput(MotorID motor, double power) {
        power = Robot.constrainPercentOutput(power);
        // getTalon(motor).set(ControlMode.PercentOutput, power);//FIXME: uh-oh
    }

    /**
     * Sets the percent output of all side motors to the specified value.
     * @param power the percent output on the range [-1, 1].
     */
    public void setSidesPercentOutput(double power) {
        this.setPercentOutput(MotorID.LEFT_1, power);
        this.setPercentOutput(MotorID.RIGHT_1, power);
    }

    // /**
    //  * Sets motion profile control mode on a primary motor using auxiliary output.
    //  * @param motor the primary motor (must be a "1" motor).
    //  * @param svmpValue the SetValueMotionProfile value to set.
    //  */
    // public void setMotionProfileValue(MotorID motor, SetValueMotionProfile svmpValue) {
    //     switch (motor) {
    //         case RIGHT_1:
    //             this.right1.set(ControlMode.MotionProfile, svmpValue.value);
    //             break;
    //         case LEFT_1:
    //             this.left1.set(ControlMode.MotionProfile, svmpValue.value);
    //             break;
    //         default:
    //             break;
    //     }
    // } //FIXME: this is gettign worse and worse...

    /**
     * Sets all drivetrain motor outputs to zero.
     */
    public void zeroMotors() {
        this.setPercentOutput(MotorID.RIGHT_1, 0);
        this.setPercentOutput(MotorID.LEFT_1, 0);
    }

    // /**
    //  * Gets the position of the sensor associated with the specified Talon.
    //  * @param motorID the Talon whose sensor position to fetch.
    //  * @return the position of the specified sensor.
    //  */
    // public int getSensorPosition(MotorID motorID) {
    //     return getTalon(motorID).getSelectedSensorPosition();
    // }
    //
    // /**
    //  * Gets the velocity of the sensor associated with the specified Talon.
    //  * @param motorID the Talon whose sensor velocity to fetch.
    //  * @return the velocity of the specified sensor.
    //  */
    // public double getSensorVelocity(MotorID motorID) {
    //     return getTalon(motorID).getSelectedSensorVelocity();
    // }
    //
    // /**
    //  * Gets current percent output of Talon.
    //  * @param motorID Talon to query.
    //  * @return percent output of talon.
    //  */
    // public double getOutputPercent(MotorID motorID) {
    //     return getTalon(motorID).getMotorOutputPercent();
    // }//FIXME: AHHHHHHHHHH!

    /**
     * Gets a motor controller with the specified ID.
     * @param motorID the ID of the Talon to fetch.
     * @return the specified Talon.
     */
    public SwerveMotor getSwerveMotor(MotorID motorID) {
        switch (motorID) {
            case RIGHT_1:
                return this.right1;
            case RIGHT_2:
                return this.right2;
            case LEFT_1:
                return this.left1;
            case LEFT_2:
                return this.left2;
            default: // this case should NEVER be reached; it is just used to prevent NPE warnings
                return getSwerveMotor(MotorID.LEFT_1);
        }
    }

    /**
     * Returns the Pigeon yaw value.
     * @return Pigeon yaw value.
     */
    public double getPigeonYaw() {
        double[] ypr = new double[3];
        this.pigeon.getYawPitchRoll(ypr);
        return ypr[0] * -1;
    }

    /**
     * Returns the Pigeon pitch value.
     * @return Pigeon pitch value.
     */
    public double getPigeonPitch() {
        double[] ypr = new double[3];
        this.pigeon.getYawPitchRoll(ypr);
        return ypr[2]; // this should be 1, but the Pigeon is mounted incorrectly
    }

    /**
     * Sets the neutral mode of the side motors to the given value.
     * @param mode The neutral mode to set the motors to.
     */
    public void setNeutralMode(NeutralMode mode) {
        // this.right1.setNeutralMode(mode);
        // this.right2.setNeutralMode(mode);
        // this.left1.setNeutralMode(mode);
        // this.left2.setNeutralMode(mode);
        //FIXME: should this still be here
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SwerveDriveWithJoystick());
    }
}
