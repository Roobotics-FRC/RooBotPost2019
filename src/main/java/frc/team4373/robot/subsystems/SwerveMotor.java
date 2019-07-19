package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.team4373.robot.RobotMap;

/**
 * A Javadoc template. TODO: Update SwerveMotor Javadoc.
 */
public class SwerveMotor {
    private WPI_TalonSRX driveMotor;
    private WPI_TalonSRX rotatorMotor;
    public SwerveMotor(Drivetrain.MotorID motorID) {
        RobotMap.MotorConfig driveMotorConfig = RobotMap.getDriveMotorConfig(motorID);
        RobotMap.MotorConfig rotatorMotorConfig = RobotMap.getRotatorMotorConfig(motorID);

        this.driveMotor = new WPI_TalonSRX(driveMotorConfig.port);
        this.rotatorMotor = new WPI_TalonSRX(rotatorMotorConfig.port);

        this.driveMotor.setInverted(driveMotorConfig.inverted);
        this.rotatorMotor.setInverted(rotatorMotorConfig.inverted);

        this.driveMotor.setNeutralMode(driveMotorConfig.neutralMode);
        this.rotatorMotor.setNeutralMode(rotatorMotorConfig.neutralMode);

        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.rotatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }
    public double getAngle() {
        return 0;//FIXME: ...obviously
    }

    public WPI_TalonSRX getDriveMotor() {
        return this.driveMotor;
    }

    public WPI_TalonSRX getRotatorMotor() {
        return this.rotatorMotor;
    }
}