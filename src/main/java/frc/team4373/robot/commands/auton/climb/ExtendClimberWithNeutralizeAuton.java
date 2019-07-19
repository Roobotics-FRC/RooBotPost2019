package frc.team4373.robot.commands.auton.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team4373.robot.commands.auton.utility.DelayAuton;

/**
 * A Javadoc template. TODO: Update ExtendClimberWithNeutralizeAuton Javadoc.
 */
public class ExtendClimberWithNeutralizeAuton extends CommandGroup {
    public ExtendClimberWithNeutralizeAuton() {
        addSequential(new DeployClimberFrontAuton());
        // addSequential(new DelayAuton(0.1));
        addSequential(new NeutralizeClimberFrontAuton());
        // addSequential(new DelayAuton(0.1));
        addSequential(new DeployClimberRearAuton());
    }
}
