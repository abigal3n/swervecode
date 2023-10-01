package frc.robot.subsytems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


 
public class SwerveModule { 

    
    //add motor model later

    //defines motors for a singular module

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    //add encoder model later

    //defines encoders for a singular module

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final CANCoder absoluteEncoder;

    //pid drive controller somehow
    //add tuning values

    private final PIDController drivePIDController = new PIDController(1,0,0);

    //pid turning trapezoidal

    private final ProfiledPIDController turningPIDController =
        new ProfiledPIDController(
            1,
            0, 
            0,
            new TrapezoidProfile.Constraints(
                2 * Math.PI,
                2 * Math.PI
            )
        );

        //add values later

    // private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(s, v);
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.11475, v, a);

    public SwerveModule(
        int driveMotorID,
        int turnMotorID,
        int turnEncoderID,
        boolean driveEncoderReversed,
        boolean turnEncoderReversed,
        float magnetOffset
        )
    {

        //add motor name
        //makes it so you can define motor channels for the modules in subsystem
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveEncoderReversed);
        turnMotor.setInverted(turnEncoderReversed);

        //add encoder name
        //makes it so you can define encoder channels for the modules in subsystem
        driveEncoder = driveMotor.getEncoder();
        
        absoluteEncoder = new CANCoder(turnEncoderID);



        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);}

        public SwerveModuleState getState() {
            return new SwerveModuleState(
                driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(
                driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));
        }

        public void setDesiredState(SwerveModuleState desiredState) {
            SwerveModuleState state =
                SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.getPosition()));

            final double driveOutput =
                drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

            final double turnOutput =
                turningPIDController.calculate(turnEncoder.getPosition(), state.angle.getRadians());

            //sets the motors to the calculated output
            driveMotor.set(driveOutput);
            turnMotor.set(turnOutput);
        }

        public void resetEncoders() {
            driveEncoder.setPosition(0);
            turnEncoder.setPosition(0);
        }
    }


