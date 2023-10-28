package frc.robot.subsytems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsytems.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

//add motor channel numbers later
public class swervesubsystem extends SubsystemBase {

    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveModule fLSwerve = new SwerveModule(0,0,0, false, false, Rotation2d.fromRadians(1.5754));
    private final SwerveModule fRSwerve = new SwerveModule(0,0,0,false ,false, Rotation2d.fromRadians(-1.2026));
    private final SwerveModule bLSwerve = new SwerveModule(0,0,0, false, false, Rotation2d.fromRadians(-2.6982));
    private final SwerveModule bRSwerve = new SwerveModule(0,0,0,false,false, Rotation2d.fromRadians(2.6952));

    XboxController primaryController = new XboxController(0);
    XboxController secondaryController = new XboxController(0);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics m_kinematics =
    new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        fLSwervesetDesiredState(swerveModuleStates[0]);
        fRSwerve.setDesiredState(swerveModuleStates[1]);
        bLSwerve.setDesiredState(swerveModuleStates[2]);
        bRSwerve.setDesiredState(swerveModuleStates[3]);
        }

    

}