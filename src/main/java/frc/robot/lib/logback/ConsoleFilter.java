package frc.robot.lib.logback;

import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.spi.ILoggingEvent;
import ch.qos.logback.core.filter.Filter;
import ch.qos.logback.core.spi.FilterReply;

public class ConsoleFilter extends Filter<ILoggingEvent> {

    @Override
    public FilterReply decide( final ILoggingEvent event ) {
        if ( event.getLevel() == Level.DEBUG ) {  
            return FilterReply.DENY;
        } else {
            return FilterReply.NEUTRAL;
        }
    }
}
