package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Example extends SubsystemBase{

    public double exampleDouble = 0;

    public Example() {
    }

    public Example(int number) {
        exampleDouble = number;
    }

    public Example(int number, int exampleDouble) {
        this.exampleDouble = exampleDouble-number;
    }

    public void exampleMethod() {
        exampleDouble = 0;
    }

    double testMethod() {
        return 1;
    }

    public void exampleMethod(int number) {
        exampleDouble = number;
    }

    public double getExampleDouble() {
        return exampleDouble;
    }
}