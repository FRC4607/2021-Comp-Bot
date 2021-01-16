package frc.robot.lib.drivers;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


/**
* The PDP class is a wrapper for the PowerDistributionPanel object which will clear any sticky faults and log the
* creation of the PowerDistributionPanel object.
*/
public class PDP {

    private static final Logger mLogger = LoggerFactory.getLogger( PDP.class );

    /**
    * This method will clear the sticky faults of the power distribution panel and log the creation of the object.
    * @param pdp PowerDistributionPanel The power distribution panel object
    * @param deviceID int The ID of the power distruution panel
    */
    public static PowerDistributionPanel createPDP ( PowerDistributionPanel pdp, int deviceID ) {
        
        pdp.clearStickyFaults();
        mLogger.info( "Created power distribution panel [{}]", deviceID );

        return pdp;
    }    

}