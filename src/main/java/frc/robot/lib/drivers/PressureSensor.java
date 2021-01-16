package frc.robot.lib.drivers;
import edu.wpi.first.wpilibj.AnalogInput;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
* The PressureSensor class uses the AnalogInput class to implement the pressure sensor connected to the analog input.
* This class is specifically setup to use the ProSense digital pressure sensor sold by automation direct.  The analog
* output of the sensor is wired to the analog input.  Care should be taken to ensure the sensor output falls within
* range of the analog input (220 Ohms works well).  To get accurate pressure reading, the this class needs two
* calibration values: the analog input voltage reading at 0 PSI and the analog input voltage reading at ~115 PSI.
* Using these two values, the pressure is calculated using: pressure = 
* (current_sensor_voltage - volts_at_0_PSI) * pressure_per_volt
*/
public class PressureSensor {
    private static final Logger mLogger = LoggerFactory.getLogger( PressureSensor.class );
    private final double mVoltsAtZeroPressure;
    private final double mPressurePerVolt;
    private AnalogInput mAnalogInput;

    /**
    * This method will sample the analog input voltage and compute the pressure using the following:
    * (current_sensor_voltage - volts_at_0_PSI) * pressure_per_volt.  The volts_at_0_PSI and pressure_per_volt values
    * need to be user-calibrated.
    * @return boolean True if an object is detected, false otherwise
    * @see {@link https://www.automationdirect.com/adc/shopping/catalog/process_control_-a-_measurement/pressure_sensors/digital_pressure_switches_-z-_transmitters/qpsh-an-42}
    */
    public double GetPressureInPSI () {
        return ( mAnalogInput.getAverageVoltage() - mVoltsAtZeroPressure ) * mPressurePerVolt; 
        //return mAnalogInput.getAverageVoltage();  // Used for calibration
    }

    /**
    * This is the PressureSensor class consructor.  The class sets up the AnalogInput class to use both oversampling and
    * averaging to produce stable data.
    * @param channel int The analog input channel of the sensor
    * @param voltsAtZeroPressure double User-calibrated analog input voltage at 0 PSI
    * @param pressurePerVolt double User-calibrated PSI per analog input voltage
    * @see {@link edu.wpi.first.wpilibj.AnalogInput}
    */
    public PressureSensor ( int channel, double voltsAtZeroPressure, double pressurePerVolt ) {
        mAnalogInput = new AnalogInput( channel );
        mVoltsAtZeroPressure = voltsAtZeroPressure;
        mPressurePerVolt = pressurePerVolt;
        mAnalogInput.setOversampleBits( 6 );
        mAnalogInput.setAverageBits( 6 );
        mLogger.info( "Created pressure sensor [{}]", channel );
    }

}
