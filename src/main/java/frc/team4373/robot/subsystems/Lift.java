package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team4373.robot.Robot;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.teleop.LiftCommand;

/**
 * A programmatic representation of the robot's lift mechanism that supports the intake.
 */
public class Lift extends Subsystem {
    private static Lift instance;

    public static Lift getInstance() {
        return instance == null ? instance = new Lift() : instance;
    }

    private WPI_TalonSRX talon1;
    private WPI_TalonSRX talon2;
    private DoubleSolenoid piston1;
    private DoubleSolenoid piston2;
    private double initialPosition;

    private Lift() {
        this.talon1 = new WPI_TalonSRX(RobotMap.LIFT_MOTOR_1);
        this.talon2 = new WPI_TalonSRX(RobotMap.LIFT_MOTOR_2);

        this.piston1 = new DoubleSolenoid(RobotMap.PCM_1_PORT,
                RobotMap.LIFT_PISTON_1_FORWARD, RobotMap.LIFT_PISTON_1_BACKWARD);
        this.piston2 = new DoubleSolenoid(RobotMap.PCM_1_PORT,
                RobotMap.LIFT_PISTON_2_FORWARD, RobotMap.LIFT_PISTON_2_BACKWARD);

        this.talon1.setNeutralMode(NeutralMode.Brake);
        this.talon2.setNeutralMode(NeutralMode.Brake);

        this.talon2.follow(talon1);

        this.talon1.setInverted(RobotMap.LIFT_MOTOR_1_INVERTED);
        this.talon2.setInverted(RobotMap.LIFT_MOTOR_2_INVERTED);

        this.talon1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
                RobotMap.TALON_TIMEOUT_MS);
        this.talon1.setSensorPhase(RobotMap.LIFT_ENCODER_PHASE);

        this.talon1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS);
        this.talon1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS);
        this.talon1.configPeakOutputForward(RobotMap.LIFT_PEAK_OUTPUT, RobotMap.TALON_TIMEOUT_MS);
        this.talon1.configPeakOutputReverse(-RobotMap.LIFT_PEAK_OUTPUT, RobotMap.TALON_TIMEOUT_MS);

        this.talon1.config_kP(RobotMap.LIFT_PID_IDX, RobotMap.LIFT_PID_GAINS.kP);

        // int absolutePosition = this.talon1.getSensorCollection().getQuadraturePosition();
        // absolutePosition &= 0xFFF;
        // if (RobotMap.LIFT_ENCODER_PHASE) absolutePosition *= -1;
        // if (RobotMap.LIFT_MOTOR_1_INVERTED) absolutePosition *= -1;
        // this.talon1.setSelectedSensorPosition(absolutePosition);
        this.initialPosition = this.talon1.getSelectedSensorPosition();
    }

    public void raise() {
        this.piston1.set(DoubleSolenoid.Value.kForward);
        this.piston2.set(DoubleSolenoid.Value.kForward);
    }

    public void lower() {
        this.piston1.set(DoubleSolenoid.Value.kReverse);
        this.piston2.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Gets whether the closed loop error on the lift is within an acceptable range
     * (i.e., whether the lift has reached the specified position).
     * @return whether the error is acceptable.
     */
    public boolean closedLoopErrorIsTolerable() {
        double error = this.talon1.getClosedLoopError();
        return error <= RobotMap.LIFT_ACCEPTABLE_CLOSED_LOOP_ERROR;
    }

    /**
     * Sets the raw percent output of the lift motors.
     * @param power the percent output to which to set the motors.
     */
    public void setPercentOutput(double power) {
        if (getSensorPosition() > this.initialPosition) {
            power = Robot.constrainPercentOutput(power);
            this.talon1.set(ControlMode.PercentOutput, power);
        }
    }

    /**
     * Sets a position setpoint in encoder units relative to the starting position.
     * @param setpoint the setpoint to set.
     */
    public void setPositionRelative(double setpoint) {
        setpoint += initialPosition;
        if (setpoint < initialPosition) setpoint = initialPosition;
        this.talon1.set(ControlMode.Position, setpoint);
    }

    /**
     * Returns the position of the encoder on the primary Talon.
     * @return the Talon's current position, in encoder units.
     */
    public double getSensorPosition() {
        return this.talon1.getSelectedSensorPosition();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new LiftCommand());
    }
}
