package frc.team4373.robot.commands.auton.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.subsystems.Climber;
import frc.team4373.robot.subsystems.ClimberDrive;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Extends the climber and sets the neutral mode of the drivetrain to coast.
 */
public class ExtendClimberAuton extends Command {
    private Climber climber;
    private Drivetrain drivetrain;

    /**
     * Constructs an ExtendClimberAuton command.
     */
    public ExtendClimberAuton() {
        requires(this.climber = Climber.getInstance());
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        setTimeout(0.5);
    }

    @Override
    protected void execute() {
        this.climber.climb();
        this.drivetrain.setNeutralMode(NeutralMode.Coast);
        System.out.println(this.getClass().getName());
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }
}