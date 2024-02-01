package frc.common.lib.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimpleNetworkTable  {
    private NetworkTable netTable;
            
    public SimpleNetworkTable(String name) {
    //    NetworkTableInstance.getDefault().setUpdateRate(Settings.kNetworkTableUpdateRate);
    //   netTable = NetworkTableInstance.getDefault().getTable(name);

    //    netTable.getInstance().setUpdateRate(Settings.kNetworkTableUpdateRate);
    //    netTable.getInstance().setServerTeam(1885);
    //    netTable.getInstance().startClientTeam(1885);
    }
    
    public void initKeys() {
      
    }
    
    public synchronized NetworkTableEntry getEntry(String key) {
      return netTable.getEntry(key);
    }
  
    public synchronized void putDouble(String key, double value) {
    	netTable.getEntry(key).setDouble(value);
    }
    
    public synchronized void putNumber(String key, Integer value) {
        netTable.getEntry(key).setNumber(value);
    }
    
    public synchronized void putNumberArray(String key, Integer[] values) {
    		netTable.getEntry(key).setNumberArray(values);
    }
    
    public synchronized void putString(String key, String value) {
    		netTable.getEntry(key).setString(value);
    }
    
    public synchronized NetworkTable getInstance() {
    		return netTable;
    }

}