package frc.team4373.robot.commands.auton.drive;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Drives a provided distance in a straight line at a given speed.
 */
public class DriveDistanceAuton extends PIDCommand {
    private PIDController distancePIDController;
    private PIDSource distanceSource;
    private PIDOutput distanceOutputLambda;

    private double setpointRelative;
    private double distancePIDOutput = 1d;
    private double robotSpeed = 0.25d;
    private double cooldownThreshold = 0.0625;

    private Drivetrain drivetrain;

    /**
     * Constructs a new DriveDistanceAuton command and initializes the secondary PID controller.
     * @param distance The distance, in inches, that the robot should drive.
     */
    public DriveDistanceAuton(double distance, double speed) {
        super("DriveDistanceAuton", RobotMap.DRIVETRAIN_ANG_PID_GAINS.kP,
                RobotMap.DRIVETRAIN_ANG_PID_GAINS.kI,
                RobotMap.DRIVETRAIN_ANG_PID_GAINS.kD);
        this.setpointRelative = distance / RobotMap.DRIVETRAIN_SIDE_ENC_UNITS_TO_IN;
        this.robotSpeed = speed;
        this.cooldownThreshold = this.robotSpeed * 0.25;
        requires(this.drivetrain = Drivetrain.getInstance());

        // Distance PID initialization
        distanceSource = new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                return;
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return getAveragePosition();
            }
        };

        distanceOutputLambda = output -> {
            this.distancePIDOutput = output;
        };
        this.distancePIDController = new PIDController(RobotMap.DRIVETRAIN_DIST_PID_GAINS.kP,
                RobotMap.DRIVETRAIN_DIST_PID_GAINS.kI, RobotMap.DRIVETRAIN_DIST_PID_GAINS.kD,
                distanceSource, distanceOutputLambda);

        // this.getPIDController().setPID(SmartDashboard.getNumber("kP-T", 0),
        //         SmartDashboard.getNumber("kI-T", 0),
        //         SmartDashboard.getNumber("kD-T", 0));
        // this.distancePIDController.setPID(SmartDashboard.getNumber("kP-D", 0),
        //         SmartDashboard.getNumber("kI-D", 0),
        //         SmartDashboard.getNumber("kD-D", 0));
    }

    @Override
    protected void initialize() {
        // Distance PID configuration
        this.distancePIDController.setOutputRange(-robotSpeed, robotSpeed);
        this.distancePIDController.setSetpoint(getAveragePosition()
                + setpointRelative);
        this.distancePIDController.enable();
        this.distancePIDOutput = robotSpeed;

        // Angular PID configuration
        this.setSetpoint(drivetrain.getPigeonYaw());
        this.getPIDController().setOutputRange(-robotSpeed, robotSpeed);
    }

    @Override
    protected double returnPIDInput() {
        return drivetrain.getPigeonYaw();
    }

    @Override
    protected void usePIDOutput(double angleOutput) {
        this.drivetrain.setPercentOutput(Drivetrain.MotorID.RIGHT_1,
                distancePIDOutput + angleOutput);
        this.drivetrain.setPercentOutput(Drivetrain.MotorID.LEFT_1,
                distancePIDOutput - angleOutput);
    }

    /**
     * Averages the positions of the left and right encoders.
     * @return the average position of the drivetrain.
     */
    private int getAveragePosition() {
        return (this.drivetrain.getSensorPosition(Drivetrain.MotorID.RIGHT_1)
                + this.drivetrain.getSensorPosition(Drivetrain.MotorID.LEFT_1)) / 2;
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(this.distancePIDOutput) < cooldownThreshold;
    }

    @Override
    protected void end() {
        this.getPIDController().reset();
        this.distancePIDController.reset();
        this.drivetrain.zeroMotors();
    }

    @Override
    protected void interrupted() {
        this.end();
    }
}