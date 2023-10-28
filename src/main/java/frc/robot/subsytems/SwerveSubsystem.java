package frc.robot.subsytems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.DriveConstants.MaxSpeed;

//add motor channel numbers later
public class SwerveSubsystem extends SubsystemBase {

    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveModule fLSwerve = new SwerveModule(0,0,0, false, false, Rotation2d.fromRadians(1.5754));
    private final SwerveModule fRSwerve = new SwerveModule(0,0,0,false ,false, Rotation2d.fromRadians(-1.2026));
    private final SwerveModule bLSwerve = new SwerveModule(0,0,0, false, false, Rotation2d.fromRadians(-2.6982));
    private final SwerveModule bRSwerve = new SwerveModule(0,0,0,false,false, Rotation2d.fromRadians(2.6952));

    XboxController primaryController = new XboxController(0);
    XboxController secondaryController = new XboxController(0);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        //TODO: DEFINE MAX SPEED
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MaxSpeed);
        fLSwerve.setDesiredState(swerveModuleStates[0]);
        fRSwerve.setDesiredState(swerveModuleStates[1]);
        bLSwerve.setDesiredState(swerveModuleStates[2]);
        bRSwerve.setDesiredState(swerveModuleStates[3]);
        }

    

}