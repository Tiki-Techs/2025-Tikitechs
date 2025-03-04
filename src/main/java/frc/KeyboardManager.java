package frc;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeyboardManager {
    
    public static Trigger f13buttonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/f13", false));
    }

    public static Trigger f14buttonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/14", false));
    }
    
    public static Trigger f15ButtonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/15", false));
    }

    public static Trigger f16ButtonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/16", false));
    }

    public static Trigger f17buttonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/17", false));
    }

    public static Trigger f18buttonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/18", false));
    }

    public static Trigger f19buttonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/19", false));
    }

    public static Trigger f20buttonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/20", false));
    }

    public static Trigger f21buttonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/21", false));
    }

    public static Trigger f22buttonTrigger(){
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/22", false));
    }
}
