package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeyboardManager {

    public static Trigger aButton() {
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/a", false));
    }

    public static Trigger bButton() {
        return new Trigger(() -> SmartDashboard.getBoolean("keyboard/b", false));
    }
}
