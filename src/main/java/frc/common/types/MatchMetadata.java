package frc.common.types;

import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

public class MatchMetadata {

    public final String mEventName;
    public final OptionalInt mLocation;
    public final Optional<Optional<Alliance>> mAlliance;
    public final MatchType mMatchType;
    public final int mReplayNumber;
    public final int mMatchNumber;
    public final int hash;

    public MatchMetadata() {
        
        if(DriverStation.isFMSAttached()) {
            mEventName = DriverStation.getEventName();
            mLocation = DriverStation.getLocation();
            mAlliance = Optional.of(DriverStation.getAlliance());;
            mMatchType = DriverStation.getMatchType();
            mReplayNumber = DriverStation.getReplayNumber();
            mMatchNumber = DriverStation.getMatchNumber();
            int i = 7;
            i += mEventName.hashCode();
            i += 7 * mMatchType.ordinal();
            i += 7 * mMatchNumber;
            hash = i;
        } else {
            mEventName = "Test";
            mLocation = OptionalInt.of(0);
            mAlliance = Optional.of(DriverStation.getAlliance());
            mMatchType = MatchType.None;
            mReplayNumber = 0;
            mMatchNumber = (int)(Math.random() * (double)Integer.MAX_VALUE);
            hash = mMatchNumber;
        }
    }
}
