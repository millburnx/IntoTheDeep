package org.firstinspires.ftc.teamcode.common.utils

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction

fun DcMotorEx.init(isForward: Boolean = true, isBrake: Boolean = false) {
    direction = if (isForward) Direction.FORWARD else Direction.REVERSE
    zeroPowerBehavior = if (isBrake) ZeroPowerBehavior.BRAKE else ZeroPowerBehavior.FLOAT
    reset()
}

fun DcMotorEx.reset() {
    mode = RunMode.STOP_AND_RESET_ENCODER
    mode = RunMode.RUN_WITHOUT_ENCODER
}