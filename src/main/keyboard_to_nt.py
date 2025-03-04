import ntcore
import keyboard
import time


def main():
    def on_action(event: keyboard.KeyboardEvent):
        if event.is_keypad:
            table.putBoolean(
                "numpad" + event.name, event.event_type == keyboard.KEY_DOWN
            )
        else:
            table.putBoolean(event.name.lower(), event.event_type == keyboard.KEY_DOWN)

    ntcoreinst = ntcore.NetworkTableInstance.getDefault()

    print("Setting up NetworkTables client")
    ntcoreinst.startClient4("KeyboardToNT")
    ntcoreinst.setServer("127.0.0.1")
    ntcoreinst.startDSClient()


    #Wait for connection
    print("Waiting for connection to NetworkTables server...")

    keyboard.hook(lambda e: on_action(e))
    keyboard.wait()

if __name__ == "__main__":
    main()