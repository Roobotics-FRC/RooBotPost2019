package frc.team4373.robot.commands.auton.drive;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.subsystems.Drivetrain;

public class StrafeDistanceAuton extends PIDCommand {
    private Drivetrain drivetrain;
    private double outputThreshold = RobotMap.AUTON_MIDDLE_WHEEL_ADJUSTMENT_SPEED * 0.1;
    private boolean finished = false;

    /**
     * Creates a new StrafeDistanceAuton command.
     */
    public StrafeDistanceAuton() {
        super("StrafeDistanceAuton", RobotMap.DRIVETRAIN_MIDDLE_PID_GAINS.kP,
                RobotMap.DRIVETRAIN_MIDDLE_PID_GAINS.kI, RobotMap.DRIVETRAIN_MIDDLE_PID_GAINS.kD);
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        this.finished = false;
        this.getPIDController().setPID(SmartDashboard.getNumber("kP-D", 0),
                SmartDashboard.getNumber("kI-D", 0),
                SmartDashboard.getNumber("kD-D", 0));
        this.getPIDController().setOutputRange(-RobotMap.AUTON_MIDDLE_WHEEL_ADJUSTMENT_SPEED,
                RobotMap.AUTON_MIDDLE_WHEEL_ADJUSTMENT_SPEED);
        this.setSetpoint(this.drivetrain.getSensorPosition(Drivetrain.MotorID.MIDDLE_1)
                + SmartDashboard.getNumber("Strafe Distance", 0)
                / RobotMap.DRIVETRAIN_MID_ENC_UNITS_TO_IN);
    }

    @Override
    protected double returnPIDInput() {
        return this.drivetrain.getSensorPosition(Drivetrain.MotorID.MIDDLE_1);
    }

    @Override
    protected void usePIDOutput(double output) {
        System.out.println("PID Output = " + output);
        System.out.println("PID Setpoint = " + this.getSetpoint());
        System.out.println("PID Cur = " + this.drivetrain.getSensorPosition(
                Drivetrain.MotorID.MIDDLE_1));
        if (Math.abs(output) < outputThreshold) {
            this.finished = true;
        } else {
            drivetrain.setPercentOutput(Drivetrain.MotorID.MIDDLE_1, output);
        }
    }

    @Override
    protected boolean isFinished() {
        return this.finished;
    }
}
