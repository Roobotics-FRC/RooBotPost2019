package frc.team4373.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * A command that controls the drivetrain via the joystick.
 */
public class DrivetrainCommand extends Command {
    private Drivetrain drivetrain;

    public DrivetrainCommand() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        // this is commented out b/c it shuts off the light ring if operator manually enables it
        // because initialize() is re-called after an interrupt
        // this.drivetrain.setLightRing(false);
    }

    @Override
    protected void execute() {
        switch (OI.getOI().getDriveJoystick().getPOV()) {
            case 315:
            case 0:
            case 45:
                drivetrain.retractMiddleWheel();
                break;
            case 135:
            case 180:
            case 225:
                drivetrain.deployMiddleWheel();
                break;
            default:
                break;
        }
        double x = OI.getOI().getDriveJoystick().rooGetX();
        double y = OI.getOI().getDriveJoystick().rooGetY();
        double z = OI.getOI().getDriveJoystick().rooGetZFiltered();
        drivetrain.setPercentOutput(Drivetrain.MotorID.MIDDLE_1, x);
        drivetrain.setPercentOutput(Drivetrain.MotorID.RIGHT_1, y + z);
        drivetrain.setPercentOutput(Drivetrain.MotorID.LEFT_1, y - z);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        this.drivetrain.zeroMotors();
    }

    @Override
    protected void interrupted() {
        this.end();
    }
}
