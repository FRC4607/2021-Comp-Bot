package frc.robot.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANError;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* This class is used as a wrapper around the CANSparkMax object in order to initialize the motor controller to known
* configuration.  Also, any sticky errors present will be logged and cleared.
* @see {@link http://www.revrobotics.com/content/sw/max/sw-docs/java/index.html}
*/
public class SparkMax {

    private static final Logger mLogger = LoggerFactory.getLogger( SparkMax.class );

    /**
    * This method is wrapper for a newly created CANSparkMax object that is intended to initialize the motor controller
    * to a know state.  The following lists what this method does and the state of the controller.
    * <p>
    * <ul>
    * <li>Sticky faults will be logged and cleared
    * <li>Reset to factory defaults
    * <li>Voltage compensation is enabled and set for 12V
    * <li>Set the idle mode to brake
    * <li>Open-loop duty-cycle output is set to 0.0
    * </ul>
    * <p>
    * @param sparkMax CANSparkMax The motor controller to initialize
    */        
    private static void SetDefaultConfig ( CANSparkMax sparkMax ) {
        CANError canError;
        long faults;

        faults = sparkMax.getStickyFaults();
        if ( faults != 0 ) {
            mLogger.warn( "Clearing SparkMax [{}] sticky faults: [{}]", sparkMax.getDeviceId(), faults );
            canError = sparkMax.clearFaults();
            if ( canError != CANError.kOk ) {
                mLogger.error( "Could not clear sticky faults due to EC: [{}]", canError.toString() );
            }  
        }
        canError = sparkMax.restoreFactoryDefaults();
        if ( canError != CANError.kOk ) {
            mLogger.error( "Could not factory reset SparkMax [{}] due to EC: [{}]", sparkMax.getDeviceId(), canError.toString() );
        }  
        canError = sparkMax.enableVoltageCompensation( 12.0 );
        if ( canError != CANError.kOk ) {
            mLogger.error( "Could not set SparkMax [{}] voltage compensation due to EC: [{}]", sparkMax.getDeviceId(), canError.toString() );
        }  
        canError = sparkMax.setIdleMode( IdleMode.kBrake );
        if ( canError != CANError.kOk ) {
            mLogger.error( "Could not set SparkMax [{}] idle mode due to EC: [{}]", sparkMax.getDeviceId(), canError.toString() );
        }        
        sparkMax.set( 0.0 );        

    }

    /**
    * Create a master CANSparkMax motor controller.
    * @param sparkMax CANSparkMax The motor controller to initialize
    */  
    public static CANSparkMax CreateSparkMax ( CANSparkMax sparkMax ) {
        SetDefaultConfig( sparkMax );
        mLogger.info( "Created master Spark Max [{}] ", sparkMax.getDeviceId() );
        return sparkMax;
    }

    /**
    * Create a follower CANSparkMax motor controller.
    * @param sparkMax CANSparkMax The motor controller to initialize 
    * @param master CANSparkMax The motor controller to follow 
    */  
    public static CANSparkMax CreateSparkMax ( CANSparkMax sparkMax, CANSparkMax master ) {
        SetDefaultConfig( sparkMax );
        sparkMax.follow( master );
        mLogger.info( "Created follower Spark Max [{}], master [{}] ", sparkMax.getDeviceId(), master.getDeviceId() );
        return sparkMax;
    }

}
