package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.team4373.robot.Robot;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.teleop.LiftCommand;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Arrays;

/**
 * A programmatic representation of the robot's lift mechanism that supports the intake.
 */
public class Lift extends Subsystem {
    private static volatile Lift instance;

    /**
     * The getter for the Lift class.
     * @return the singleton Lift object.
     */
    public static Lift getInstance() {
        if (instance == null) {
            synchronized (Lift.class) {
                if (instance == null) {
                    instance = new Lift();
                }
            }
        }
        return instance;
    }

    private WPI_TalonSRX talon1;
    private WPI_TalonSRX talon2;
    private DoubleSolenoid piston1;
    private Potentiometer poten;
    private double upperSafety;
    private double lowerSafety;

    private Lift() {
        this.talon1 = new WPI_TalonSRX(RobotMap.LIFT_MOTOR_1);
        this.talon2 = new WPI_TalonSRX(RobotMap.LIFT_MOTOR_2);

        this.piston1 = new DoubleSolenoid(RobotMap.PCM_2_PORT,
                RobotMap.LIFT_PISTON_FORWARD, RobotMap.LIFT_PISTON_BACKWARD);

        this.poten = new AnalogPotentiometer(RobotMap.PTN_CHANNEL,
                RobotMap.LIFT_MAX_POTEN_VALUE, RobotMap.LIFT_INITIAL_ANG_OFFSET);

        this.talon1.setNeutralMode(NeutralMode.Brake);
        this.talon2.setNeutralMode(NeutralMode.Brake);

        this.talon2.follow(talon1);

        this.talon1.setInverted(RobotMap.LIFT_MOTOR_1_INVERTED);
        this.talon2.setInverted(RobotMap.LIFT_MOTOR_2_INVERTED);

        if (Robot.isSecondBot) {
            upperSafety = RobotMap.LIFT_MAXIMUM_SAFE_ANGLE_2;
            lowerSafety = RobotMap.LIFT_MINIMUM_SAFE_ANGLE_2;
        } else {
            upperSafety = RobotMap.LIFT_MAXIMUM_SAFE_ANGLE_1;
            lowerSafety = RobotMap.LIFT_MINIMUM_SAFE_ANGLE_1;
        }
    }

    /**
     * Telescopes the lift up.
     */
    public void telescope() {
        this.piston1.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Retracts the telescoping pistons.
     */
    public void retract() {
        this.piston1.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Sets the raw percent output of the lift motors with ramping. Positive power means moving up.
     * @param power the percent output to which to set the motors.
     */
    public void setPercentOutputRamping(double power) {
        double curPower = this.talon1.get();
        double diff = power - curPower;
        diff = Math.abs(diff) > RobotMap.LIFT_MAXIMUM_RAMP_INCREASE
                ? Math.copySign(RobotMap.LIFT_MAXIMUM_RAMP_INCREASE, diff) : diff;
        setPercentOutputRaw(curPower + diff);
    }

    /**
     * Sets the raw percent output of lift motors with NO RAMPING. BE CAREFUL. Positive power = up.
     * @param power the power to set.
     */
    public void setPercentOutputRaw(double power) {
        power = Robot.constrainPercentOutput(power);
        double angle = getPotenValue();
        if (angle <= lowerSafety) {
            this.talon1.set(power > 0 ? power : 0);
        } else if (angle >= upperSafety) {
            this.talon1.set(power < 0 ? power : 0);
        } else {
            this.talon1.set(power);
        }
    }

    /**
     * Gets the absolute value of the potentiometer
     * and adjusts to compensate for hardware irregularity on second bot.
     * @return absolute value of potentiometer.
     */
    public double getPotenValue() {
        if (Robot.isSecondBot) {
            // Since potentiometer is backwards on second bot, reverse [max, min] to [min, max]
            return -(poten.get() - RobotMap.LIFT_MAXIMUM_SAFE_ANGLE_2)
                    + RobotMap.LIFT_MINIMUM_SAFE_ANGLE_2;
        } else {
            return poten.get();
        }
    }

    public boolean isTelescoped() {
        return this.piston1.get() == DoubleSolenoid.Value.kForward;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new LiftCommand());
    }
}