package frc.team4373.robot.commands.auton.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.subsystems.Climber;
import frc.team4373.robot.subsystems.ClimberDrive;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Extends the font climber and sets neutral modes.
 */
public class DeployClimberFrontAuton extends Command {
    private Climber climber;
    private Drivetrain drivetrain;

    /**
     * Constructs a command to retract the climber.
     */
    public DeployClimberFrontAuton() {
        requires(this.climber = Climber.getInstance());
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        setTimeout(0.5);
    }

    @Override
    protected void execute() {
        this.climber.deployFront();
        this.drivetrain.setNeutralMode(NeutralMode.Coast);
        System.out.println(this.getClass().getName());
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }
}