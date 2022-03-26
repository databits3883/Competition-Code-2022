package frc.robot.subsystems;

import java.util.ArrayList;

public class SafetyOverrideRegistry {
    private static SafetyOverrideRegistry instance;
    public static SafetyOverrideRegistry getInstance(){
        if(instance == null){
            instance = new SafetyOverrideRegistry();
        }
        return instance;
    }

    private SafetyOverrideRegistry(){
        overridables = new ArrayList<>();
    }


    private ArrayList<SafetyOverridable> overridables;

    /**
     * Adds a manual safety overridable to the registry and enables its safety mode
     * @param overridable the SafetyOverridable to add
     */
    public void register(SafetyOverridable overridable){
        overridables.add(overridable);
        overridable.enableSafety();
    }

    public void disableSafety(){
        for(SafetyOverridable o : overridables){
            o.disableSafety();
        }
    }
    public void enableSafety(){
        for(SafetyOverridable o : overridables){
            o.enableSafety();
        }
    }

}
