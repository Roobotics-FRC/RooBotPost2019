package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team4373.robot.Robot;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.teleop.DriveStraightCommand;

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

    public enum TalonID {
        RIGHT_1, RIGHT_2, LEFT_1, LEFT_2, MIDDLE_1, MIDDLE_2
    }

    private WPI_TalonSRX right1;
    private WPI_TalonSRX right2;
    private WPI_TalonSRX left1;
    private WPI_TalonSRX left2;
    private WPI_TalonSRX middle1; // note: for now, no PID on the center motors
    private WPI_TalonSRX middle2;
    private PigeonIMU pigeon;
    private DoubleSolenoid piston;
    private Relay lightRingRelay;
    private boolean middleWheelDeployed = true; // TODO: is this a valid assumption?
    private double initialAngle;

    private Drivetrain() {
        this.right1 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_1);
        this.right2 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_2);
        this.left1 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_MOTOR_LEFT_1);
        this.left2 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_MOTOR_LEFT_2);
        this.middle1 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_MOTOR_MIDDLE_1);
        this.middle2 = new WPI_TalonSRX(RobotMap.DRIVETRAIN_MOTOR_MIDDLE_2);

        this.pigeon = new PigeonIMU(this.right2);
        this.piston = new DoubleSolenoid(RobotMap.PCM_1_PORT,
                RobotMap.DRIVETRAIN_PISTON_FORWARD, RobotMap.DRIVETRAIN_PISTON_BACKWARD);

        this.lightRingRelay = new Relay(RobotMap.LIGHT_RING_RELAY_CHANNEL);

        this.left1.setNeutralMode(NeutralMode.Brake);
        this.left2.setNeutralMode(NeutralMode.Brake);
        this.right1.setNeutralMode(NeutralMode.Brake);
        this.right2.setNeutralMode(NeutralMode.Brake);
        this.middle1.setNeutralMode(NeutralMode.Brake);
        this.middle2.setNeutralMode(NeutralMode.Brake);

        this.right2.follow(right1);
        this.left2.follow(left1);
        this.middle2.follow(middle1);

        this.left1.setInverted(RobotMap.DRIVETRAIN_MOTOR_LEFT_1_INVERTED);
        this.left2.setInverted(RobotMap.DRIVETRAIN_MOTOR_LEFT_2_INVERTED);
        this.right1.setInverted(RobotMap.DRIVETRAIN_MOTOR_RIGHT_1_INVERTED);
        this.right2.setInverted(RobotMap.DRIVETRAIN_MOTOR_RIGHT_2_INVERTED);
        this.middle1.setInverted(RobotMap.DRIVETRAIN_MOTOR_MIDDLE_1_INVERTED);
        this.middle2.setInverted(RobotMap.DRIVETRAIN_MOTOR_MIDDLE_2_INVERTED);

        this.right1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.right1.setSensorPhase(RobotMap.DRIVETRAIN_RIGHT_ENCODER_PHASE);
        this.left1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.left1.setSensorPhase(RobotMap.DRIVETRAIN_LEFT_ENCODER_PHASE);
        this.middle1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.middle1.setSensorPhase(RobotMap.DRIVETRAIN_MIDDLE_SENSOR_PHASE);

        this.initialAngle = getPigeonYaw();
    }

    /**
     * Returns the angle from the given inputs, relative to the field, where 0 is forward and clockwise counts up.
     */
    public double calculateAngle(double x, double y) {
        return (((360 - (Math.toDegrees(Math.atan(y/x) + (x >= 0 ? 0 : Math.PI)) - 90)) % 360) + 360) % 360;
        //FIXME: This might be Math.atan2(double y, double x) // plus toDegrees // 0lus some more stuff
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
    public void rotate(TalonID motor, double power) {
        //rotate the given motor.
    }

    public void rotate(double power) {
        for (TalonID t: TalonID.values()) {
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
    public void setPercentOutput(TalonID motor, double power) {
        power = Robot.constrainPercentOutput(power);
        if (isMiddle(motor)) {
            if (!middleWheelDeployed) power = 0;
        }
        getTalon(motor).set(ControlMode.PercentOutput, power);
    }

    /**
     * Sets the percent output of all side motors to the specified value.
     * @param power the percent output on the range [-1, 1].
     */
    public void setSidesPercentOutput(double power) {
        this.setPercentOutput(TalonID.LEFT_1, power);
        this.setPercentOutput(TalonID.RIGHT_1, power);
    }

    /**
     * Sets motion profile control mode on a primary motor using auxiliary output.
     * @param motor the primary motor (must be a "1" motor).
     * @param svmpValue the SetValueMotionProfile value to set.
     */
    public void setMotionProfileValue(TalonID motor, SetValueMotionProfile svmpValue) {
        switch (motor) {
            case RIGHT_1:
                this.right1.set(ControlMode.MotionProfile, svmpValue.value);
                break;
            case LEFT_1:
                this.left1.set(ControlMode.MotionProfile, svmpValue.value);
                break;
            default:
                break;
        }
    }

    /**
     * Turns the light ring on and off.
     * @param enable whether to enable the light ring.
     */
    public void setLightRing(boolean enable) {
        if (enable) {
            this.lightRingRelay.set(Relay.Value.kForward);
        } else {
            this.lightRingRelay.set(Relay.Value.kOff);
        }
    }

    /**
     * Whether the light ring is currently on.
     * @return whether the ring is on.
     */
    public boolean getLightRingEnabled() {
        return this.lightRingRelay.get() == Relay.Value.kForward;
    }

    /**
     * Sets all drivetrain motor outputs to zero.
     */
    public void zeroMotors() {
        this.setPercentOutput(TalonID.RIGHT_1, 0);
        this.setPercentOutput(TalonID.LEFT_1, 0);
        this.setPercentOutput(TalonID.MIDDLE_1, 0);
    }

    /**
     * Retracts middle wheel using pistons.
     */
    public void retractMiddleWheel() {
        this.piston.set(DoubleSolenoid.Value.kReverse);
        this.middleWheelDeployed = false;
    }

    /**
     * Deploys the middle wheel using pistons.
     */
    public void deployMiddleWheel() {
        this.piston.set(DoubleSolenoid.Value.kForward);
        this.middleWheelDeployed = true;
    }

    /**
     * Returns whether the middle wheel is currently deployed.
     * @return Whether or not the middle wheel is deployed.
     */
    public boolean isMiddleWheelDeployed() {
        return middleWheelDeployed;
    }

    /**
     * Gets the position of the sensor associated with the specified Talon.
     * @param talonID the Talon whose sensor position to fetch.
     * @return the position of the specified sensor.
     */
    public int getSensorPosition(TalonID talonID) {
        return getTalon(talonID).getSelectedSensorPosition();
    }

    /**
     * Gets the velocity of the sensor associated with the specified Talon.
     * @param talonID the Talon whose sensor velocity to fetch.
     * @return the velocity of the specified sensor.
     */
    public double getSensorVelocity(TalonID talonID) {
        return getTalon(talonID).getSelectedSensorVelocity();
    }

    /**
     * Gets current percent output of Talon.
     * @param talonID Talon to query.
     * @return percent output of talon.
     */
    public double getOutputPercent(TalonID talonID) {
        return getTalon(talonID).getMotorOutputPercent();
    }

    /**
     * Gets a motor controller with the specified ID.
     * @param talonID the ID of the Talon to fetch.
     * @return the specified Talon.
     */
    public WPI_TalonSRX getTalon(TalonID talonID) {
        switch (talonID) {
            case RIGHT_1:
                return this.right1;
            case RIGHT_2:
                return this.right2;
            case LEFT_1:
                return this.left1;
            case LEFT_2:
                return this.left2;
            case MIDDLE_1:
                return this.middle1;
            case MIDDLE_2:
                return this.middle2;
            default: // this case should NEVER be reached; it is just used to prevent NPE warnings
                return this.right1;
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
        this.right1.setNeutralMode(mode);
        this.right2.setNeutralMode(mode);
        this.left1.setNeutralMode(mode);
        this.left2.setNeutralMode(mode);
    }

    /**
     * Returns whether the specified Talon is one of the middle ones.
     * @param talonID the Talon ID to process.
     * @return whether the Talon is connected to a middle wheel motor.
     */
    private boolean isMiddle(TalonID talonID) {
        return talonID == TalonID.MIDDLE_1 || talonID == TalonID.MIDDLE_2;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveStraightCommand()); //FIXME: Maybe not on first bot, needed on 2
    }
}
