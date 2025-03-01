package frc.lib.util;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Access keyboard and mouse from dashboard
 */
public class KeyboardAndMouse {

    private static KeyboardAndMouse keyboard = new KeyboardAndMouse();
    private double deltaX = 0;
    private double deltaY = 0;
    private double x = 0;
    private double y = 0;

    private KeyboardAndMouse() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        instance.addListener(instance.getTopic("/input/keyboardDown"),
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                String s = ev.valueData.value.getString();
                if (s.isEmpty()) { // an empty value is sent to allow repeat values
                    return;
                }
                this.keys.put(s, true);
                System.out.println(s);
            });
        instance.addListener(instance.getTopic("/input/keyboardUp"),
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (ev) -> {
                String s = ev.valueData.value.getString();
                if (s.isEmpty()) { // an empty value is sent to allow repeat values
                    return;
                }
                this.keys.put(s, false);
            });
    }

    public static KeyboardAndMouse getInstance() {
        return keyboard;
    }

    private Map<String, Boolean> keys = new HashMap<>();
   
    /**
     *
     * @param name key name (lowercase from javascript)
     * @return keypress trigger
     */
    public Trigger key(String name) {
        return new Trigger(() -> {
            return this.keys.getOrDefault(name, false);
        });
    }

    /**
     *
     * @param button mouse button (from javascript). Usually left click is 0, right click is 2.
     * @return mousepress trigger
     */
    public Trigger mouse(int button) {
        return key("mouse" + button);
    }


    }


