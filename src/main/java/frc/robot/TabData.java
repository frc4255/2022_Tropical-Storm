package frc.robot;


import java.util.HashMap;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;


public class TabData {

    ShuffleboardTab tab;

    HashMap<String, NetworkTableEntry> entries;

    int defaultValue = 0;

    public TabData(String tabName){
        tab = Shuffleboard.getTab(tabName);

        entries = new HashMap<String, NetworkTableEntry>();
    }


    /**
     * Will retrieve the entry with the given name, or make one if it doesn't exist.
     * When making new entries, will use a default value of 0.
     * @param entryName Name of the entry you want to access.
     * @return A NetworkTableEntry object.
     */
    public NetworkTableEntry getEntry(String entryName){
        if(entries.containsKey(entryName)){
            return entries.get(entryName);
        } else{
            NetworkTableEntry entry = tab.add(entryName, defaultValue).getEntry();
            entries.put(entryName, entry);
            return entry;
        }
    }

    /**
     * Will retrieve the value from a specified entry.
     * This value will usually need to be converted to another data type using its built in methods.
     * @param entryName Name of the entry you want to access.
     * @return A NetworkTableValue object.
     */
    public NetworkTableValue getEntryData(String entryName){
        NetworkTableEntry entry = getEntry(entryName);

        return entry.getValue();
    }

    /**
     * Will update the specified entry with a given valid value.
     * @param entryName Name of the entry you want to access.
     * @param value Value to update entry with
     */
    public void updateEntry(String entryName, Object value){

        NetworkTableEntry entry = getEntry(entryName);

        if(String.class.equals(value.getClass())){
            entry.setString((String) value);
        } else{
            entry.setValue(value);
        }
    }
}