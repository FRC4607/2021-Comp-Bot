package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.lib.drivers.TalonSRX;

public class FlywheelSubsystem extends SubsystemBase {
    private final WPI_TalonSRX mMaster;
    private final WPI_TalonSRX mFollower;

    private double mTarget;

    // 100ms * 10 * 60 to go from r/100ms to rpm
    private int conversionFactor = 600;

    public FlywheelSubsystem(WPI_TalonSRX master, WPI_TalonSRX follower) {
        mMaster = master;
        mFollower = follower;

        mMaster.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative,
                                              Constants.FLYWHEEL.PID_IDX, Constants.GLOBAL.CAN_TIMEOUT_MS );
		mMaster.setSensorPhase( false );
        mMaster.setNeutralMode( NeutralMode.Coast ); 
        mMaster.setInverted( true );
        mMaster.configNominalOutputForward( 0, Constants.GLOBAL.CAN_LONG_TIMEOUT_MS );
        mMaster.configNominalOutputReverse( 0, Constants.GLOBAL.CAN_LONG_TIMEOUT_MS );
        mMaster.configPeakOutputForward( 1, Constants.GLOBAL.CAN_LONG_TIMEOUT_MS );
        mMaster.configPeakOutputReverse( 0, Constants.GLOBAL.CAN_LONG_TIMEOUT_MS );
        mFollower.setInverted( false ); 
        mFollower.follow( mMaster );
        mFollower.setNeutralMode( NeutralMode.Coast );

        mMaster.config_kP( Constants.FLYWHEEL.PID_IDX, 0.006, Constants.GLOBAL.CAN_TIMEOUT_MS );
		mMaster.config_kI( Constants.FLYWHEEL.PID_IDX, 0, Constants.GLOBAL.CAN_TIMEOUT_MS );
		mMaster.config_kD( Constants.FLYWHEEL.PID_IDX, 0, Constants.GLOBAL.CAN_TIMEOUT_MS );
        mMaster.config_kF( Constants.FLYWHEEL.PID_IDX, 0.0198, Constants.GLOBAL.CAN_TIMEOUT_MS );

        mMaster.getSelectedSensorVelocity();
        
        setOpenLoop();
    }

    @Override
    public void periodic() {
        double mCurrentVelocity_RPM = mMaster.getSelectedSensorVelocity( Constants.FLYWHEEL.PID_IDX ) / Constants.FLYWHEEL.SENSOR_UNITS_PER_ROTATION * conversionFactor;
        double mError_RPM = mTarget - mCurrentVelocity_RPM;
        SmartDashboard.putNumber( "FlyWheel Target (RPM)", mTarget );   
        SmartDashboard.putNumber( "FlyWheel RPM", mCurrentVelocity_RPM );
        SmartDashboard.putNumber( "Flywheel Error (RPM)", mError_RPM);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setOpenLoop() {
        mMaster.set(ControlMode.PercentOutput, 0.0);

        mMaster.configContinuousCurrentLimit( Constants.CURRENT_LIMIT.TALON_AMPS_LIMIT );     
        mFollower.configContinuousCurrentLimit( Constants.CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mMaster.configPeakCurrentLimit( Constants.CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mFollower.configPeakCurrentLimit( Constants.CURRENT_LIMIT.TALON_AMPS_LIMIT );
        mMaster.configPeakCurrentDuration( Constants.GLOBAL.TALON_CURRENT_LIMIT_TIMEOUT_MS );
        mFollower.configPeakCurrentDuration( Constants.GLOBAL.TALON_CURRENT_LIMIT_TIMEOUT_MS );
        mMaster.enableCurrentLimit( true );
        mFollower.enableCurrentLimit( true );
    }

    public double getEncoderVelocity() {
        return mMaster.getSelectedSensorVelocity() * 10 / 4096;
    }

    public void setClosedLoop(double target) {
        mTarget = target;
        mMaster.set(ControlMode.Velocity, target / conversionFactor * Constants.FLYWHEEL.SENSOR_UNITS_PER_ROTATION);
    }

    public static FlywheelSubsystem create() {
        WPI_TalonSRX master = TalonSRX.createTalonSRXWithEncoder(new WPI_TalonSRX(Constants.FLYWHEEL.MASTER_ID));
        WPI_TalonSRX follower = TalonSRX.createTalonSRX(new WPI_TalonSRX(Constants.FLYWHEEL.FOLLOWER_ID), master);
        return new FlywheelSubsystem(master, follower);
    }
}
