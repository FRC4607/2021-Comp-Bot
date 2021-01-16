package frc.robot.lib.drivers;
import edu.wpi.first.wpilibj.AnalogInput;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* The Photoeye class uses the AnalogInput class to implement the photoeye or beam-break sensors.  This class is
* specifically setup for PNP-type sensor outputs and wiring.  This means a detected object will pull the signal
* to ground, otherwise, it will be ~4-Volts (depending on how it is wired up).
*/
public class Photoeye {
    private static final Logger mLogger = LoggerFactory.getLogger( Photoeye.class );
    private static final double mThreshold = 1.0;
    private AnalogInput mAnalogInput;

    /**
    * This method will sample the analog input voltage and compare to a threshold in order to determine if object has
    * been detected. 
    * @return boolean True if an object is detected, false otherwise
    */
    public boolean IsPhotoeyeClosed () {
        return mAnalogInput.getAverageVoltage() > mThreshold ?  false : true;  // PNP
    }

    /**
    * This is the Photoeye class consructor.  The class sets up the AnalogInput class to use both oversampling and
    * averaging to produce stable data.
    * @param channel int The analog input channel of the sensor
    * @see {@link edu.wpi.first.wpilibj.AnalogInput}
    */
    public Photoeye ( int channel ) {
        mAnalogInput = new AnalogInput( channel );
        mAnalogInput.setOversampleBits( 6 );
        mAnalogInput.setAverageBits( 6 );
        mLogger.info( "Created photoeye [{}]", channel );
    }

}
