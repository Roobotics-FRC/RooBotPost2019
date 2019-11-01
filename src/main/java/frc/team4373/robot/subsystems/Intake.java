package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team4373.robot.Robot;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.teleop.IntakeCommand;

/**
 * A programmatic representation of the robot's intake mechanism for both hatch panels and cargo.
 */
public class Intake extends Subsystem {

    private WPI_TalonSRX leftTalon;
    private WPI_TalonSRX rightTalon;
    private DoubleSolenoid hatchPiston;
    private DoubleSolenoid deployPiston1;
    private DigitalInput limitSwitch;

    private static final DoubleSolenoid.Value DEPLOYED = DoubleSolenoid.Value.kReverse;
    private static final DoubleSolenoid.Value RETRACTED = DoubleSolenoid.Value.kForward;

    private static final DoubleSolenoid.Value RETAIN_HATCH = DoubleSolenoid.Value.kForward;
    private static final DoubleSolenoid.Value RELEASE_HATCH = DoubleSolenoid.Value.kReverse;

    private static volatile Intake instance;

    /**
     * The getter for the Intake class.
     * @return the singleton Intake object.
     */
    public static Intake getInstance() {
        if (instance == null) {
            synchronized (Intake.class) {
                if (instance == null) {
                    instance = new Intake();
                }
            }
        }
        return instance;
    }

    private Intake() {
        leftTalon = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_LEFT);
        rightTalon = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_RIGHT);

        hatchPiston = new DoubleSolenoid(RobotMap.PCM_2_PORT,
                RobotMap.INTAKE_PISTON_HATCH_FORWARD, RobotMap.INTAKE_PISTON_HATCH_BACKWARD);
        deployPiston1 = new DoubleSolenoid(RobotMap.PCM_1_PORT,
                RobotMap.INTAKE_PISTON_DEPLOYMENT_FORWARD,
                RobotMap.INTAKE_PISTON_DEPLOYMENT_BACKWARD);

        limitSwitch = new DigitalInput(RobotMap.INTAKE_LIMIT_SWITCH_CHANNEL);

        this.leftTalon.setNeutralMode(NeutralMode.Brake);
        this.rightTalon.setNeutralMode(NeutralMode.Brake);

        this.leftTalon.setInverted(RobotMap.INTAKE_MOTOR_LEFT_INVERTED);
        this.rightTalon.setInverted(RobotMap.INTAKE_MOTOR_RIGHT_INVERTED);

    }

    public void collectCargo() {
        setLeftTalon(RobotMap.INTAKE_MOTOR_OUTPUT);
        setRightTalon(-RobotMap.INTAKE_MOTOR_OUTPUT);
    }

    public void releaseCargo() {
        setLeftTalon(-RobotMap.INTAKE_MOTOR_OUTPUT);
        setRightTalon(RobotMap.INTAKE_MOTOR_OUTPUT);
    }

    public void neutralizeCargoMotors() {
        setLeftTalon(0);
        setRightTalon(0);
    }

    public void collectHatch() {
        hatchPiston.set(RETAIN_HATCH);
    }

    public void releaseHatch() {
        hatchPiston.set(RELEASE_HATCH);
    }

    private void setLeftTalon(double power) {
        power = Robot.constrainPercentOutput(power);
        this.leftTalon.set(ControlMode.PercentOutput, power);
    }

    private void setRightTalon(double power) {
        power = Robot.constrainPercentOutput(power);
        this.rightTalon.set(ControlMode.PercentOutput, power);
    }

    public void deploy() {
        this.deployPiston1.set(DEPLOYED);
    }

    public void retract() {
        this.deployPiston1.set(RETRACTED);
    }

    public boolean isDeployed() {
        return this.deployPiston1.get() == DEPLOYED;
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public boolean getHatchPanelIntakeRetaining() {
        return this.hatchPiston.get() == RETAIN_HATCH;
    }

    public double getRightMotorPower() {
        return this.rightTalon.get();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new IntakeCommand());
    }
}