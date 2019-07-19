package frc.team4373.robot.commands.auton.drive;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Strafes in order to line up with vision targets using data provided via NetworkTables.
 */
public class MiddleWheelAdjusterAuton extends PIDCommand {
    private Drivetrain drivetrain;
    private boolean finished;
    private boolean initiallyDeployed;
    private int sampleCount = 0;
    private double distanceSum = 0;
    private boolean readyForPID = false;
    private long waitStart = 0;
    private double outputThreshold = RobotMap.AUTON_MIDDLE_WHEEL_ADJUSTMENT_SPEED * 0.1;

    /**
     * Constructs a new auton command to align the robot with the vision target using the H-wheel.
     */
    public MiddleWheelAdjusterAuton() {
        super("MiddleWheelAdjusterAuton", RobotMap.DRIVETRAIN_MIDDLE_PID_GAINS.kP,
                RobotMap.DRIVETRAIN_MIDDLE_PID_GAINS.kI, RobotMap.DRIVETRAIN_MIDDLE_PID_GAINS.kD);
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        this.drivetrain.setLightRing(true);
        resetState(true);
        this.initiallyDeployed = drivetrain.isMiddleWheelDeployed();
        this.getPIDController().setOutputRange(-RobotMap.AUTON_MIDDLE_WHEEL_ADJUSTMENT_SPEED,
                RobotMap.AUTON_MIDDLE_WHEEL_ADJUSTMENT_SPEED);
        this.drivetrain.deployMiddleWheel();
        // this.getPIDController().setPID(SmartDashboard.getNumber("kP-D", 0),
        //         SmartDashboard.getNumber("kI-D", 0),
        //         SmartDashboard.getNumber("kD-D", 0));
    }

    @Override
    protected double returnPIDInput() {
        return this.drivetrain.getSensorPosition(Drivetrain.MotorID.MIDDLE_1);
    }

    @Override
    protected void usePIDOutput(double output) {
        if (sampleCount < RobotMap.VISION_SAMPLE_COUNT) { // polling state
            SmartDashboard.putString("MWA State", "Polling");
            if (System.currentTimeMillis() > waitStart + 500) {
                distanceSum += SmartDashboard.getNumber("lateral_distance_to_target", 0);
                System.out.println(SmartDashboard.getNumber("lateral_distance_to_target", 0));
                ++sampleCount;
            }
        } else if (!readyForPID) { // setpoint setting state
            SmartDashboard.putString("MWA State", "Setting");
            double setpointInches = distanceSum / RobotMap.VISION_SAMPLE_COUNT;
            if (Math.abs(setpointInches) < RobotMap.ALLOWABLE_LATERAL_OFFSET_FROM_VIS_TARGET) {
                this.finished = true;
            } else {
                this.setSetpoint(this.drivetrain.getSensorPosition(Drivetrain.MotorID.MIDDLE_1)
                        + setpointInches / RobotMap.DRIVETRAIN_MID_ENC_UNITS_TO_IN);
                readyForPID = true;
            }
        } else { // PID execution state
            SmartDashboard.putString("MWA State", "Executing");
            if (Math.abs(output) < outputThreshold) {
                // this.finished = true;
                resetState(false);
            } else {
                drivetrain.setPercentOutput(Drivetrain.MotorID.MIDDLE_1, output);
            }
        }
    }

    /**
     * Reset state.
     * @param isInitial if it's the initial reset.
     */
    private void resetState(boolean isInitial) {
        if (isInitial) {
            this.waitStart = 0;
        } else {
            this.waitStart = System.currentTimeMillis();
        }
        this.sampleCount = 0;
        this.distanceSum = 0;
        this.readyForPID = false;
        this.finished = false;
    }

    @Override
    protected boolean isFinished() {
        return this.finished;
    }

    @Override
    protected void end() {
        this.drivetrain.setLightRing(false);
        this.drivetrain.zeroMotors();
        if (initiallyDeployed) {
            drivetrain.deployMiddleWheel();
        } else {
            drivetrain.retractMiddleWheel();
        }
    }

    @Override
    protected void interrupted() {
        this.end();
    }
}
