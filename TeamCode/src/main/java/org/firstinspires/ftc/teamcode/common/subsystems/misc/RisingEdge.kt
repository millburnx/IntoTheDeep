package org.firstinspires.ftc.teamcode.common.subsystems.misc

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.common.utils.GamepadSRL

class RisingEdge(val gamepadSRL: GamepadSRL, val button: GamepadKeys.Button, val runnable: () -> Unit) :
    SubsystemBase() {
    var lastState: Boolean = false;
    override fun periodic() {
        val currentState = gamepadSRL.gamepad.getGamepadButton(button).get()
        if (lastState == false && currentState == true) {
            runnable()
        }
        lastState = currentState
    }
}