#pragma once

#include <frc/Encoder.h>

#include <frc/drive/MecanumDrive.h>

#include <frc/geometry/Pose2d.h>

#include <frc/geometry/Rotation2d.h>

#include <frc/interfaces/Gyro.h>

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc/motorcontrol/PWMSparkMax.h>

#include <frc/smartdashboard/Field2d.h>

#include <frc/XboxController.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

#include <frc/DataLogManager.h>

#include <frc2/command/SubsystemBase.h>

#include <wpi/DataLog.h>

#include <networktables/NetworkTable.h>

#include <networktables/NetworkTableEntry.h>

#include <networktables/NetworkTableInstance.h>

#include <ctre/Phoenix.h>

#include "DiffSwerveModule.h"

class DrivetrainSubsystem : public frc2::SubsystemBase
{
public:
    DrivetrainSubsystem(wpi::log::DataLog &log);

    void Periodic() override;

    void SimulationPeriodic() override;

    /**
     * Drives the robot at given x, y and theta speeds.
     * Linear speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     */
    void Drive(
        units::meters_per_second_t xSpeed,
        units::meters_per_second_t ySpeed,
        units::radians_per_second_t rot);

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    void ResetEncoders();

    /**
     * Sets the drive MotorControllers to a power from -1 to 1.
     *
     * @param desiredStates array for target states
     */
    void SetModuleStates(wpi::array<frc::SwerveModuleState, DriveConstants::kModuleCount> &desiredStates);

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    units::degree_t GetHeading() const;

    /**
     * Zeroes the heading of the robot to Zero.
     */
    void ZeroHeading();

    /**
     * Sets the heading of the robot to offset parameter.
     *
     * @param heading target heading
     */
    void SetOffsetHeading(int heading);

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    double GetTurnRate();

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    frc::Pose2d GetPose();

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    void ResetOdometry(frc::Pose2d pose);

    frc::SwerveDriveKinematics<DriveConstants::kModuleCount> kDriveKinematics{
        DriveConstants::FrontLeftModule::translation,
        DriveConstants::FrontRightModule::translation,
        DriveConstants::RearLeftModule::translation,
        DriveConstants::RearRightModule::translation};

    /** turn off both motors */
    void MotorsOff();

    /* toggles field centric mode */
    void ToggleFieldCentric();

    /**
     * set field centric mode
     *
     * @param fieldCentric target state
     */
    void SetFieldCentric(bool fieldCentric);

    /**
     * gyro-assisted movement
     *
     * @param x speed
     * @param y speed
     */
    void GyroCrab(double x, double y, double desiredAngle);

    /* set wheel offsets for modules */
    void SetWheelOffsets();

    /* loads wheel offsets on modules */
    void LoadWheelOffsets();

    DiffSwerveModule m_frontLeft;
    DiffSwerveModule m_frontRight;
    DiffSwerveModule m_rearLeft;
    DiffSwerveModule m_rearRight;

    DiffSwerveModule *moduleArray[DriveConstants::kModuleCount] = {
        &m_frontLeft,
        &m_frontRight,
        &m_rearLeft,
        &m_rearRight};

private:
    // The gyro sensor
    WPI_Pigeon2 m_pigeon{
        0,
        GlobalConstants::kRIOBus};

    BasePigeonSimCollection m_pigeonSim;

    // Odometry class for tracking robot pose
    frc::SwerveDriveOdometry<4> m_odometry;

    frc::Field2d m_field;

    frc::Pose2d m_lastPose;

    wpi::log::DataLog &m_log;

    // Field Centric Control as an option
    bool m_fieldCentric;
    double m_zero;
    double m_currentYaw = 0;
    int m_counter = 0; // Counter for USER button zero reset
    units::radians_per_second_t m_vr;
};
