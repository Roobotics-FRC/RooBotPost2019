package frc.team4373.robot.commands.auton.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.subsystems.Climber;
import frc.team4373.robot.subsystems.ClimberDrive;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Extends the climber and sets the neutral mode of the drivetrain to coast.
 */
public class ExtendClimberWithPitchAuton extends Command {
    private Climber climber;
    private Drivetrain drivetrain;

    private long lastPitchTime = -1;
    private boolean initialDeployOccurred = false;
    private boolean finished = false;
    private double startingPitch = 0;
    private static final double TOLERABLE_PITCH = 1;
    private static final long COOLDOWN = 1000;


    /**
     * Constructs an ExtendClimberAuton command.
     */
    public ExtendClimberWithPitchAuton() {
        requires(this.climber = Climber.getInstance());
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        this.initialDeployOccurred = false;
        this.finished = false;
        this.startingPitch = drivetrain.getPigeonPitch();
        setTimeout(30); // FIXME: Undo if used at competition
    }

    @Override
    protected void execute() {
        long now = System.currentTimeMillis();
        if (!this.initialDeployOccurred) {
            // this.climber.deployFront();
            //TODO: below
            this.climber.climb();
            this.initialDeployOccurred = true;
        } else {
            double pitch = this.drivetrain.getPigeonPitch();
            if (pitch - this.startingPitch > TOLERABLE_PITCH) {
                this.climber.neutralizeFront();
                this.climber.deployRear();
                lastPitchTime = now;
                this.finished = false;
            } else if (pitch + this.startingPitch < -TOLERABLE_PITCH) {
                this.climber.neutralizeRear();
                this.climber.deployFront();
                lastPitchTime = now;
                this.finished = false;
            } else if (this.lastPitchTime > -1) {
                this.climber.climb();
                if (now > this.lastPitchTime + COOLDOWN) {
                    this.finished = true;
                }
            }
            this.drivetrain.setNeutralMode(NeutralMode.Coast);
        }
        System.out.println(this.getClass().getName());
    }

    @Override
    protected boolean isFinished() {
        // return this.isTimedOut() || this.finished;
        return this.isTimedOut();
    }
}