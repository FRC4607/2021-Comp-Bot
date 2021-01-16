package frc.robot.lib.drivers;

import frc.robot.Constants.GLOBAL;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* This class is used as a wrapper around the WPI_VictorSPX object in order to initialize the motor controller to known
* configuration.  Also, any sticky errors present will be logged and cleared.
* @see {@link https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html}
*/
public class VictorSPX {

    private static final Logger mLogger = LoggerFactory.getLogger( VictorSPX.class );

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
    private static void setDefaultConfig( WPI_VictorSPX victor ) {
        StickyFaults faults = new StickyFaults();

        victor.getStickyFaults( faults );
        if ( faults.hasAnyFault() ) {
            mLogger.warn( "Clearing VictorSPX [{}] sticky faults: [{}]", victor.getDeviceID(), faults.toString() );
            final ErrorCode clearStickyFaults = victor.clearStickyFaults( GLOBAL.CAN_LONG_TIMEOUT_MS );
            if ( clearStickyFaults != ErrorCode.OK ) {
                mLogger.error( "Could not clear sticky faults due to EC: [{}]", clearStickyFaults );
            }  
        }
    
        final ErrorCode configFactoryDefault = victor.configFactoryDefault();
        if ( configFactoryDefault != ErrorCode.OK ) {
            mLogger.error( "Could not factory reset VictorSPX [{}] due to EC: [{}]", victor.getDeviceID(), configFactoryDefault );
        }  

        final ErrorCode configVoltageCompSaturation = victor.configVoltageCompSaturation( 12.0, GLOBAL.CAN_LONG_TIMEOUT_MS );
        if ( configVoltageCompSaturation != ErrorCode.OK ) {
            mLogger.error( "Could not set VictorSPX [{}] voltage compensation due to EC: [{}]", victor.getDeviceID(), configVoltageCompSaturation );
        }  
        victor.enableVoltageCompensation( true );
    }

    /**
    * Create a master WPI_VictorSPX motor controller
    * @param victor WPI_VictorSPX The motor controller to initialize
    */  
    public static WPI_VictorSPX createVictorSPX ( WPI_VictorSPX victor ) {
        setDefaultConfig( victor );
        victor.set( ControlMode.PercentOutput, 0.0 );
        mLogger.info(" Created leader VictorSPX [{}]", victor.getDeviceID() );
        return victor;
    }
  
    /**
    * Create a follower WPI_VictorSPX motor controller
    * @param victor WPI_VictorSPX The motor controller to initialize 
    * @param master WPI_TalonSRX The motor controller to follow 
    */ 
    public static WPI_VictorSPX createVictorSPX ( WPI_VictorSPX victor, WPI_TalonSRX master ) {
        setDefaultConfig( victor );
        victor.follow( master );
        mLogger.info( "Created follower VictorSPX [{}], master [{}]", victor.getDeviceID(), master.getDeviceID() );
        return victor;
    }

}
