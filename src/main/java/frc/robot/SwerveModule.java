// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardwareTalonSRX.ControlMode;
import frc.robot.lib.chart.Chart;
import frc.robot.lib.log.LogHelper;

import static frc.robot.lib.MathUtil.radiansToDegrees;

public class SwerveModule implements LogHelper {
    private static final double kModuleMaxAngularVelocity = 2 * Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final HardwareTalonSRX m_driveMotor;
    private final HardwareTalonSRX m_turningMotor;

    private final CANCoder m_turningEncoder;
    private final String name;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            10,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 1.8);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    private GenericEntry anglePosition;
    private GenericEntry angleSetpoint;
    private GenericEntry angleError;
    private GenericEntry angleVoltage;
    private GenericEntry drivePosition;
    private GenericEntry driveVelocity;
    private GenericEntry driveSetpoint;
    private GenericEntry driveError;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel
     * @param turningMotorChannel
     * @param turningEncoderChannel
     */
    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderChannel,
            double angleOffset,
            String name) {

        this.name = name;

        m_driveMotor = HardwareTalonSRX.talonSRX(driveMotorChannel, true, NeutralMode.Brake);
        m_driveMotor.setScale(2048, 6.12, 0.3141592654); // 2048 ticks per rev, 6.12:1 gear ratio, 0.2pi m/rev
        m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        m_driveMotor.configContinuousCurrentLimit(40, 10);
        m_driveMotor.configPeakCurrentLimit(80, 10);
        m_driveMotor.configPeakCurrentDuration(100, 10);
        m_driveMotor.enableCurrentLimit(true);

        m_turningMotor = HardwareTalonSRX.talonSRX(turningMotorChannel, true, NeutralMode.Brake);
        m_turningMotor.configContinuousCurrentLimit(40, 10);
        m_turningMotor.configPeakCurrentLimit(80, 10);
        m_turningMotor.configPeakCurrentDuration(100, 10);
        m_turningMotor.enableCurrentLimit(true);
        m_turningEncoder = new CANCoder(turningEncoderChannel);
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorDirection = false;
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.magnetOffsetDegrees = angleOffset;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        m_turningEncoder.configAllSettings(config);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);

        ShuffleboardTab tab = Shuffleboard.getTab("Angle");
        ShuffleboardLayout layout = tab.getLayout(name, BuiltInLayouts.kList).withSize(3, 3)
        .withPosition(0, 0);
        anglePosition = layout.add("Position", 0).getEntry();
        angleSetpoint = layout.add("Setpoint", 0).getEntry();
        angleError = layout.add("Error", 0).getEntry();
        angleVoltage = layout.add("Voltage", 0).getEntry();

        tab = Shuffleboard.getTab("Drive");
        layout = tab.getLayout(name, BuiltInLayouts.kList).withSize(3, 3);
        drivePosition = layout.add("Position", 0).getEntry();
        driveVelocity = layout.add("Velocity", 0).getEntry();
        driveSetpoint = layout.add("Setpoint", 0).getEntry();
        driveError = layout.add("Error", 0).getEntry();

        Chart.register(() -> radiansToDegrees(getAngleSetpoint()), "%s/angle/target", name);
        Chart.register(() -> radiansToDegrees(getAnglePosition()), "%s/angle/actual", name);
        Chart.register(() -> radiansToDegrees(getAngleError()), "%s/angle/error", name);
        // GetBusVoltage() causes a lot of errors :(
        //Chart.register(() -> m_turningMotor.getOutputVoltage(), "%s/angle/volts", name);

        Chart.register(() -> getDriveSetpoint(), "%s/drive/speed/target", name);
        Chart.register(() -> getDriveVelocity(), "%s/drive/speed/actual", name);
        Chart.register(() -> getDrivePositionError(), "%s/drive/position/error", name);
        Chart.register(() -> getDrivePosition(), "%s/drive/position/actual", name);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveMotor.getSpeed(), new Rotation2d(m_turningEncoder.getAbsolutePosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_driveMotor.getPosition(), new Rotation2d(m_turningEncoder.getAbsolutePosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(m_turningEncoder.getAbsolutePosition()));

        SmartDashboard.putNumber(String.format("%s desired angle", name), desiredState.angle.getDegrees());
        SmartDashboard.putNumber(String.format("%s optimized angle", name),
        state.angle.getDegrees());

        // Calculate tihe drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getSpeed(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        double absolutePosition = m_turningEncoder.getAbsolutePosition();
        double targetAngle = state.angle.getRadians();
        final double turnOutput = m_turningPIDController.calculate(absolutePosition,
                targetAngle);

        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.set(ControlMode.Voltage, driveOutput + driveFeedforward);
        m_turningMotor.set(ControlMode.Voltage, turnOutput + turnFeedforward);
    }

    public double getAnglePosition() {
        return m_turningEncoder.getAbsolutePosition();
    }

    public double getAngleSetpoint() {
        return m_turningPIDController.getSetpoint().position;
    }

    public double getAngleError() {
        return m_turningPIDController.getPositionError();
    }

    public double getDrivePosition() {
        return m_driveMotor.getPosition();
    }

    public double getDriveVelocity() {
        return m_driveMotor.getSpeed();
    }

    public double getDriveSetpoint() {
        return m_drivePIDController.getSetpoint();
    }

    public double getDrivePositionError() {
        return m_drivePIDController.getPositionError();
    }

    public void updateShuffleboard() {
        anglePosition.setDouble(radiansToDegrees(getAnglePosition()));
        angleSetpoint.setDouble(radiansToDegrees(getAngleSetpoint()));
        angleError.setDouble(radiansToDegrees(getAngleError()));
        angleVoltage.setDouble(m_turningMotor.getOutputVoltage());

        SmartDashboard.putData(m_drivePIDController);
        SmartDashboard.putData(m_turningPIDController);

        drivePosition.setDouble(getDrivePosition());
        driveVelocity.setDouble(getDriveVelocity());
        driveSetpoint.setDouble(getDriveSetpoint());
        driveError.setDouble(getDrivePositionError());
    }

    @Override
    public String getName() {
        return "SwerveModule";
    }

}
