package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;



public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance;

    private final WPI_TalonSRX m_leftLeader;
    private final WPI_TalonSRX m_leftFollower;
    private final WPI_TalonSRX m_rightLeader;
    private final WPI_TalonSRX m_rightFollower;

    private final MotorControllerGroup m_leftGroup;
    private final MotorControllerGroup m_rightGroup;

    private final AHRS ahrs;

    private final DifferentialDriveKinematics m_kinematics;

    private final DifferentialDriveOdometry m_odometry;

    private final PIDController m_leftPIDController;
    private final PIDController m_rightPIDController;
    private final SimpleMotorFeedforward m_feedforward;

    private Field2d field;
    private DifferentialDrivetrainSim robotDriveSim;

    
    public Drivetrain(){
        m_leftLeader = new WPI_TalonSRX(DriveConstants.DRIVE_MOTOR_LEFT_LEADER_ID);
        m_leftFollower = new WPI_TalonSRX(DriveConstants.DRIVE_MOTOR_LEFT_FOLLOWER_ID);
        m_rightLeader = new WPI_TalonSRX(DriveConstants.DRIVE_MOTOR_RIGHT_LEADER_ID);
        m_rightFollower = new WPI_TalonSRX(DriveConstants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID);
        m_leftGroup = 
        new MotorControllerGroup(m_leftLeader, m_leftFollower);
        m_rightGroup =
        new MotorControllerGroup(m_rightLeader, m_rightFollower);

        ahrs = new AHRS(SPI.Port.kMXP);

        m_kinematics =
        new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth);

        m_leftPIDController = new PIDController(DriveConstants.kP, 0, 0);
        m_rightPIDController = new PIDController(DriveConstants.kP, 0, 0);
        
        m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter);

        if(RobotBase.isSimulation()){
            robotDriveSim = new DifferentialDrivetrainSim(
                DriveConstants.DRIVE_CHAR, 
                DriveConstants.GEARBOX, 
                DriveConstants.DRIVE_GEARING, 
                DriveConstants.kTrackWidth, 
                DriveConstants.kWheelRadius, 
                null
            );
        }

        // m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), new Pose2d(6,13.5, new Rotation2d()));
        m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());

        // SET EVERYTHING UP
        m_leftLeader.setNeutralMode(Constants.DriveConstants.driveMode);
        m_leftFollower.setNeutralMode(Constants.DriveConstants.driveMode);
        m_rightLeader.setNeutralMode(Constants.DriveConstants.driveMode);
        m_rightFollower.setNeutralMode(Constants.DriveConstants.driveMode);

        m_rightGroup.setInverted(true);
        ahrs.zeroYaw();

        field = new Field2d();
        SmartDashboard.putData("Field", field);
        
        resetEncoders();
        
    }

    public void periodic(){
        m_odometry.update(
            ahrs.getRotation2d(), 
            getLeftDistance(), 
            getRightDistance());
            field.setRobotPose(getPose());


    }

    public void simulationPeriodic(){

        robotDriveSim.setInputs(
            m_leftLeader.getMotorOutputVoltage(),
            m_rightLeader.getMotorOutputVoltage()
        );
        robotDriveSim.update(0.002);

		m_leftLeader.getSimCollection().setQuadratureRawPosition((int)(robotDriveSim.getLeftPositionMeters()/DriveConstants.ENCODER_DISTANCE_PER_MARK));
		m_leftLeader.getSimCollection().setQuadratureVelocity((int)(robotDriveSim.getLeftVelocityMetersPerSecond()/(DriveConstants.ENCODER_DISTANCE_PER_MARK*10)));
		m_rightLeader.getSimCollection().setQuadratureRawPosition(-(int)(robotDriveSim.getRightPositionMeters()/DriveConstants.ENCODER_DISTANCE_PER_MARK));
		m_rightLeader.getSimCollection().setQuadratureVelocity(-(int)(robotDriveSim.getRightVelocityMetersPerSecond()/(DriveConstants.ENCODER_DISTANCE_PER_MARK*10)));

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(robotDriveSim.getHeading().getDegrees());
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(getLeftRate(), getRightRate());
    }

    public void resetOdometry(Pose2d pose){
        resetEncoders();
        m_odometry.resetPosition(pose, ahrs.getRotation2d());
    }
    
    public void resetEncoders(){
        this.m_leftLeader.setSelectedSensorPosition(0);
        this.m_rightLeader.setSelectedSensorPosition(0);

    }

    public double getAverageEncoderDistance(){
        return (getLeftDistance()+getRightRate())/2.0;
    }

    public void zeroHeading(){
        ahrs.reset();
    }

    public double getHeading(){
        return ahrs.getRotation2d().getDegrees();
    }

    public double getTurnRate(){
        return ahrs.getRate();
    }

    /**
   * 
   * Gets the speed in m/s of the right side of the robot
   * @return double
   */
  public double getLeftRate(){
    // number of ticks per 100 ms -> m/s
    return m_leftLeader.getSelectedSensorVelocity() * (2 * Math.PI * Constants.DriveConstants.kWheelRadius / Constants.DriveConstants.kEncoderResolution);
  }

  /**
   * 
   * Gets the speed in m/s of the right side of the robot
   * @return double
   */
  public double getRightRate(){
    // number of ticks per 100 ms -> m/s
    return m_rightLeader.getSelectedSensorVelocity() * (2 * Math.PI * Constants.DriveConstants.kWheelRadius / Constants.DriveConstants.kEncoderResolution);
  }

  /**
   * Gets the distance travelled by the left encoder
   * @return double
   */
  public double getLeftDistance(){
    return (m_leftLeader.getSelectedSensorPosition()/Constants.DriveConstants.kEncoderResolution) * (2 * Math.PI * Constants.DriveConstants.kWheelRadius);
  }

  /**
   * Gets the distance travelled by the the right encoder
   * @return double
   */
  public double getRightDistance(){
    return (m_rightLeader.getSelectedSensorPosition()/Constants.DriveConstants.kEncoderResolution) * (2 * Math.PI * Constants.DriveConstants.kWheelRadius);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed*3, 0.0, rot*3));
    setSpeeds(wheelSpeeds);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(getLeftRate(), speeds.leftMetersPerSecond);
    final double rightOutput =  
        m_rightPIDController.calculate(getRightRate(), speeds.rightMetersPerSecond);
    
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);

}

public void setVolts(double leftVolts, double rightVolts){
    m_leftGroup.setVoltage(leftVolts);
    m_rightGroup.setVoltage(rightVolts);
    
}

/** Updates the field-relative position. */
public void updateOdometry() {
    m_odometry.update(
        ahrs.getRotation2d(), getLeftDistance(), getRightDistance());
}

public static synchronized Drivetrain getInstance() {
    if (instance == null) {
        instance = new Drivetrain();
    }
    return instance;
}

    
}
