package frc.team4373.robot.commands.auton.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.subsystems.Climber;
import frc.team4373.robot.subsystems.ClimberDrive;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Retracts the climber and sets the neutral mode of the drivetrain to brake.
 *
 * <p>Should only be used for testing.
 */
public class NeutralizeClimberAuton extends Command {
    private Climber climber;
    private Drivetrain drivetrain;

    /**
     * Constructs a command to retract the climber.
     */
    public NeutralizeClimberAuton() {
        requires(this.climber = Climber.getInstance());
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        setTimeout(0.5);
    }

    @Override
    protected void execute() {
        this.climber.neutralizeRear();
        this.climber.neutralizeFront();
        this.drivetrain.setNeutralMode(NeutralMode.Brake);
        System.out.println(this.getClass().getName());
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }
}