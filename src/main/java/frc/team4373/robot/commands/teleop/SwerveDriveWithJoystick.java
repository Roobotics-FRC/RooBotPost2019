package frc.team4373.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * A Javadoc template. TODO: Update SwerveDriveWithJoystick Javadoc.
 */
public class SwerveDriveWithJoystick extends Command {
    private Drivetrain drivetrain;

    public SwerveDriveWithJoystick() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {
        double x = OI.getOI().getDriveJoystick().rooGetX();
        double y = OI.getOI().getDriveJoystick().rooGetY();
        double z = OI.getOI().getDriveJoystick().rooGetZFiltered();
        double angle = drivetrain.calculateAngle(x, y);
        drivetrain.rotate(angle > drivetrain.getWheelAngle() ? 0.3 : -0.3);

        //TODO: rotating
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
