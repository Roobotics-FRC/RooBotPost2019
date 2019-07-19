package frc.team4373.robot.input;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.auton.ClearSubsystemsCommandGroup;
import frc.team4373.robot.commands.auton.climb.*;
import frc.team4373.robot.commands.auton.drive.SimpleMiddleWheelAdjusterAuton;
import frc.team4373.robot.commands.auton.lift.SetLiftAuton;
import frc.team4373.robot.commands.teleop.DrivetrainCommand;
import frc.team4373.robot.commands.teleop.ToggleLightRingCommand;
import frc.team4373.robot.input.filters.FineGrainedPiecewiseFilter;
import frc.team4373.robot.input.filters.XboxAxisFilter;

/**
 * OI provides access to operator interface devices.
 */
public class OI {
    private static volatile OI oi = null;
    private RooJoystick<FineGrainedPiecewiseFilter> driveJoystick;
    private RooJoystick<XboxAxisFilter> operatorJoystick;

    private OI() {
        this.driveJoystick =
                new RooJoystick<>(RobotMap.DRIVE_JOYSTICK_PORT, new FineGrainedPiecewiseFilter());
        this.operatorJoystick =
                new RooJoystick<>(RobotMap.OPERATOR_JOYSTICK_PORT, new XboxAxisFilter());

        new JoystickButton(driveJoystick, 7).whenPressed(new DeployClimberFrontAuton());
        new JoystickButton(driveJoystick, 9).whenPressed(new NeutralizeClimberFrontAuton());
        new JoystickButton(driveJoystick, 11).whenPressed(new RetractClimberFrontAuton());

        new JoystickButton(driveJoystick, 8).whenPressed(new DeployClimberRearAuton());
        new JoystickButton(driveJoystick, 10).whenPressed(new NeutralizeClimberRearAuton());
        new JoystickButton(driveJoystick, 12).whenPressed(new RetractClimberRearAuton());

        new JoystickButton(driveJoystick, 5).whenPressed(new ExtendClimberAuton());
        new JoystickButton(driveJoystick, 3).whenPressed(new NeutralizeClimberAuton());
        new JoystickButton(driveJoystick, 2).whenPressed(new RetractClimberAuton());

        // new JoystickButton(driveJoystick, 1).whenPressed(new ExtendClimberWithPitchAuton());
        new JoystickButton(driveJoystick, 1).whenPressed(new ExtendClimberWithNeutralizeAuton());


        new JoystickButton(driveJoystick, 4).whenPressed(new DrivetrainCommand());

        new JoystickButton(driveJoystick, 6).whenPressed(new ClearSubsystemsCommandGroup());
    }

    /**
     * The getter for the OI singleton.
     *
     * @return The static OI singleton object.
     */
    public static OI getOI() {
        if (oi == null) {
            synchronized (OI.class) {
                if (oi == null) {
                    oi = new OI();
                }
            }
        }
        return oi;
    }

    /**
     * Gets the drive joystick controlling the robot.
     * @return The drive joystick controlling the robot.
     */
    public RooJoystick getDriveJoystick() {
        return this.driveJoystick;
    }

    /**
     * Gets the operator joystick controlling the robot.
     * @return The operator joystick controlling the robot.
     */
    public RooJoystick getOperatorJoystick() {
        return this.operatorJoystick;
    }
}