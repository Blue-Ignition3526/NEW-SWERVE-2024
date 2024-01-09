package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase{
    /**
     * Front Left Swerve Module
     */
    private final SwerveModule m_frontLeft;

    /**
     * Front Right Swerve Module
     */
    private final SwerveModule m_frontRight;
    
    /**
     * Back Left Swerve Module
     */
    private final SwerveModule m_backLeft;

    /**
     * Back Right Swerve Module
     */
    private final SwerveModule m_backRight;

    /**
     * NavX Gyro (Over MXP I2C)
     */
    private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);

    /**
     * Odometry class for tracking robot pose on the field
     */
    public final SwerveDriveOdometry m_odometry;

    /**
     * Current chasis speeds
     */
    private ChassisSpeeds m_chassisSpeeds;
    
    /**
     * If the current chasis speeds are robot relative or field relative
     */
    private boolean drivingRobotRelative = true;

    /**
     * Create a new Swerve Drivetrain
     * @param m_frontLeft Front Left Swerve Module
     * @param m_frontRight Front Right Swerve Module
     * @param m_backLeft Back Left Swerve Module
     * @param m_backRight Back Right Swerve Module
     */
    public SwerveDrive(SwerveModule m_frontLeft, SwerveModule m_frontRight, SwerveModule m_backLeft, SwerveModule m_backRight) {
        this.m_frontLeft = m_frontLeft;
        this.m_frontRight = m_frontRight;
        this.m_backLeft = m_backLeft;
        this.m_backRight = m_backRight;

        // Reset the gyro to 0 degrees
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                m_gyro.reset();
                Thread.sleep(1000);
                m_gyro.zeroYaw();
            } catch (Exception e) {}
        }).start();

        // Update the odometry with the initial positions of the swerve modules
        this.m_odometry = new SwerveDriveOdometry(
            Constants.Swerve.Physical.m_swerveDriveKinematics,
            getRotation2d(),
            new SwerveModulePosition[]{
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );

        // Configure the autonomous builder
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPosition,
            this::getRobotRelativeChassisSpeeds,
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                Constants.Swerve.Autonomous.kTranslatePIDConstants,
                Constants.Swerve.Autonomous.kRotatePIDConstants,
                Constants.Swerve.Autonomous.kMaxSpeedMetersPerSecond,
                Constants.Swerve.Physical.kTrackWidth / 2,
                new ReplanningConfig()
            ),
            () -> {
                if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
                return false;
            },
            this
        );
    }

    /**
     * Get the current gyro angle as a Rotation2d object
     * NOT CONTINUOUS (0 to 360 degrees / 0 to 2pi radians)
     * @return Rotation2d
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(this.m_gyro.getAngle() % 360));
    }

    /**
     * Get the current swerve module state for each module
     * @return SwerveModuleState[]
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    /**
     * Tries to update the odometry (if it fails, it will print the error and still return the odometry)
     * @return SwerveDriveOdometry
     */
    public SwerveDriveOdometry getOdometry() {
        try {
            m_odometry.update(
                getRotation2d(),
                new SwerveModulePosition[]{
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
                }
            );
        } catch (Exception e) {
            System.out.println("Error updating odometry: " + e);
        }
        return m_odometry;
    }

    /**
     * Get the current pose of the robot
     * @return Pose2d
     */
    public Pose2d getPose() {
        return getOdometry().getPoseMeters();
    }

    /**
     * Reset the pose of the robot
     * @param Pose2d
     */
    public void resetPosition(Pose2d pose) {
        m_odometry.resetPosition(this.getRotation2d(), new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, pose);
    }

    /**
     * Get the current ROBOT RELATIVE ChassisSpeeds
     * @return ChassisSpeeds
     */
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        if (this.drivingRobotRelative) {
            return this.m_chassisSpeeds;
        } else {
            return ChassisSpeeds.fromFieldRelativeSpeeds(m_chassisSpeeds, getRotation2d());
        }
    }

    /**
     * Set the speeds of each swerve module (4 modules)
     * @param desiredStates The desired states of each swerve module (Array of 4 SwerveModuleState objects)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.Physical.kMaxSpeedMetersPerSecond);
        m_frontLeft.setState(desiredStates[0]);
        m_frontRight.setState(desiredStates[1]);
        m_backLeft.setState(desiredStates[2]);
        m_backRight.setState(desiredStates[3]);
    }

    /**
     * Stops all swerve modules
     */
    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    /**
     * Drive the robot using the given speeds (calculate the states for each swerve module and apply it)
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void drive(ChassisSpeeds speeds) {
        this.m_chassisSpeeds = speeds;
        SwerveModuleState[] m_moduleStates = Constants.Swerve.Physical.m_swerveDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(m_moduleStates);
    }

    /**
     * Drive the robot relative to the field
     * @param xSpeed X speed in meters per second
     * @param ySpeed Y speed in meters per second
     * @param rotSpeed Rotation speed in radians per second
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = false;
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, this.getRotation2d()));
    }

    /**
     * Drive the robot relative to the field
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    /**
     * Drive the robot relative to the robot's current position
     * @param xSpeed X speed in meters per second
     * @param ySpeed Y speed in meters per second
     * @param rotSpeed Rotation speed in radians per second
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    /**
     * Drive the robot relative to the field
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }

    @Override
    public void periodic() {}
}
