// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.lib.log.LogHelper;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain implements LogHelper {
    public static final double kMaxSpeed = 3;
    public static final double kMaxAngularSpeed =  Math.PI; // 1 rotation per second

    private final Translation2d m_frontLeftLocation = new Translation2d(0.235, 0.235);
    private final Translation2d m_frontRightLocation = new Translation2d(0.235, -0.235);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.235, 0.235);
    private final Translation2d m_backRightLocation = new Translation2d(-0.235, -0.235);

    private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 42,
            88/*-272*/, "frontLeft");
    private final SwerveModule m_frontRight = new SwerveModule(3, 4, 43,
            -130/*-310*/, "frontRight");
    private final SwerveModule m_backLeft = new SwerveModule(5, 6, 44,
            -73/*-70*/, "backLeft");
    private final SwerveModule m_backRight = new SwerveModule(7, 8, 45,
            -14/*-194*/, "backRight");

    private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

    public Drivetrain() {
        m_gyro.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));     
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void updateShuffleboard() {
        m_frontLeft.updateShuffleboard();
        m_frontRight.updateShuffleboard();
        m_backLeft.updateShuffleboard();
        m_backRight.updateShuffleboard();
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });
    }

    @Override
    public String getName() {
        return "drive";
    }
}
