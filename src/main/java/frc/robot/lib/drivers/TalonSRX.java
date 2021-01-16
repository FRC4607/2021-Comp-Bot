package frc.robot.lib.drivers;

import frc.robot.Constants.GLOBAL;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* This class is used as a wrapper around the WPI_TalonSRX object in order to initialize the motor controller to known
* configuration.  Also, any sticky errors present will be logged and cleared.
* @see {@link https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html}
*/
public class TalonSRX {

    private static final Logger mLogger = LoggerFactory.getLogger( TalonSRX.class );

    /**
    * This method is wrapper for a newly created WPI_TalonSRX object that is intended to initialize the motor controller
    * to a know state.  The following lists what this method does and the state of the controller.
    * <p>
    * <ul>
    * <li>Sticky faults will be logged and cleared
    * <li>Reset to factory defaults
    * <li>Voltage compensation is enabled and set for 12V
    * </ul>
    * <p>
    * @param talon WPI_TalonSRX The motor controller to initialize
    */ 
    private static void setDefaultConfig ( WPI_TalonSRX talon ) {
        StickyFaults faults = new StickyFaults();

        talon.getStickyFaults( faults );
        if ( faults.hasAnyFault() ) {
            mLogger.warn( "Clearing TalonSRX [{}] sticky faults: [{}]", talon.getDeviceID(), faults.toString() );
            final ErrorCode clearStickyFaults = talon.clearStickyFaults( GLOBAL.CAN_LONG_TIMEOUT_MS );
            if ( clearStickyFaults != ErrorCode.OK ) {
                mLogger.error( "Could not clear sticky faults due to EC: [{}]", clearStickyFaults );
            }  
        }
        final ErrorCode configFactoryDefault = talon.configFactoryDefault();
        if ( configFactoryDefault != ErrorCode.OK ) {
            mLogger.error( "Could not factory reset TalonSRX [{}] due to EC: [{}]", talon.getDeviceID(), configFactoryDefault );
        }  

        final ErrorCode configVoltageCompSaturation = talon.configVoltageCompSaturation( 12.0, GLOBAL.CAN_LONG_TIMEOUT_MS );
        if ( configVoltageCompSaturation != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] voltage compensation due to EC: [{}]", talon.getDeviceID(), configVoltageCompSaturation );
                }  
        talon.enableVoltageCompensation( true );
    }

    /**
    * This method is wrapper for a WPI_TalonSRX object which will setup the motor controller for using the CTRE Mag
    * Encoder for feeback.
    * <p>
    * <ul>
    * <li>Set the sensor to CTRE Mag Encoder (relative)
    * <li>Set velocity measurement period to 50ms
    * <li>Set velocity measuremeent window to 1
    * <li>Set closed-loop ramp-rate to 0   
    * </ul>
    * <p>
    * @param talon WPI_TalonSRX The motor controller to setup with CTRE Mag Encoder
    */  
    private static void setEncoderConfig ( WPI_TalonSRX talon ) {
        final ErrorCode configSelectedFeedbackSensor = talon.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100 );
        if ( configSelectedFeedbackSensor != ErrorCode.OK ) {
            mLogger.error( "Could not detect encoder due EC: [{}]", configSelectedFeedbackSensor );
        }
        final ErrorCode configVelocityMeasurementPeriod = talon.configVelocityMeasurementPeriod( VelocityMeasPeriod.Period_50Ms, GLOBAL.CAN_LONG_TIMEOUT_MS );
        if ( configVelocityMeasurementPeriod != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] voltage compensation due to EC: [{}]", talon.getDeviceID(), configVelocityMeasurementPeriod );
        }
        final ErrorCode configVelocityMeasurementWindow = talon.configVelocityMeasurementWindow( 1, GLOBAL.CAN_LONG_TIMEOUT_MS );
        if ( configVelocityMeasurementWindow != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] velocity measurement window due to EC: [{}]", talon.getDeviceID(), configVelocityMeasurementWindow );
        }
        final ErrorCode configClosedloopRamp = talon.configClosedloopRamp( 0.0, GLOBAL.CAN_LONG_TIMEOUT_MS );
        if ( configClosedloopRamp != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] closed loop ramp due to EC: [{}]", talon.getDeviceID(), configClosedloopRamp );
        }
    }

    /**
    * Create a master WPI_TalonSRX motor controller
    * @param talon WPI_TalonSRX The motor controller to initialize
    */  
    public static WPI_TalonSRX createTalonSRX ( WPI_TalonSRX talon ) {
        setDefaultConfig( talon );
        final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod( StatusFrameEnhanced.Status_2_Feedback0, 20, GLOBAL.CAN_LONG_TIMEOUT_MS );
        if ( setStatusFramePeriod != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod );
        }  
        talon.set( ControlMode.PercentOutput, 0.0 );
        mLogger.info( "Created master TalonSRX [{}]", talon.getDeviceID() );
        return talon;
    }

    /**
    * Create a follower WPI_TalonSRX motor controller
    * @param talon WPI_TalonSRX The motor controller to initialize 
    * @param master WPI_TalonSRX The motor controller to follow 
    */  
    public static WPI_TalonSRX createTalonSRX ( WPI_TalonSRX talon, WPI_TalonSRX master ) {
        setDefaultConfig( talon );
 
        final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod( StatusFrameEnhanced.Status_2_Feedback0, 160, GLOBAL.CAN_LONG_TIMEOUT_MS );
        if ( setStatusFramePeriod != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod );
        }  
        talon.follow( master);
        mLogger.info( "Created follower TalonSRX [{}], master [{}]", talon.getDeviceID(), master.getDeviceID() );
        return talon;
    }

    /**
    * Create a master WPI_TalonSRX motor controller with a CTRE Mag Encoder
    * @param talon WPI_TalonSRX The motor controller to initialize
    */  
    public static WPI_TalonSRX createTalonSRXWithEncoder ( WPI_TalonSRX talon ) {
        setDefaultConfig( talon );
        final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod( StatusFrameEnhanced.Status_2_Feedback0, 20, GLOBAL.CAN_LONG_TIMEOUT_MS );
        if ( setStatusFramePeriod != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod );
        }  
        talon.set(ControlMode.PercentOutput, 0.0);
        setEncoderConfig( talon );
        mLogger.info( "Created master TalonSRX [{}] with an encoder", talon.getDeviceID() );
        return talon;
    }

    /**
    * Create a follower WPI_TalonSRX motor controller with a CTRE Mag Encoder
    * @param talon WPI_TalonSRX The motor controller to initialize 
    * @param master WPI_TalonSRX The motor controller to follow 
    */  
    public static WPI_TalonSRX createTalonSRXWithEncoder ( WPI_TalonSRX talon, WPI_TalonSRX master) {
        setDefaultConfig( talon );
        final ErrorCode setStatusFramePeriod = talon.setStatusFramePeriod( StatusFrameEnhanced.Status_2_Feedback0, 20, GLOBAL.CAN_LONG_TIMEOUT_MS );
        if ( setStatusFramePeriod != ErrorCode.OK ) {
            mLogger.error( "Could not set TalonSRX [{}] status frame period due to EC: [{}]", talon.getDeviceID(), setStatusFramePeriod );
        }
        talon.follow( master );
        setEncoderConfig( talon );
        mLogger.info( "Created follower TalonSRX [{}] with a CTRE Mag encoder, master [{}]", talon.getDeviceID(), master.getDeviceID() );
        return talon;
    }

}