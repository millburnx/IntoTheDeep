package org.firstinspires.ftc.teamcode.common.utils

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

fun DcMotorEx.init(isForward: Boolean = true, isBrake: Boolean = false) {
    direction = if (isForward) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
    zeroPowerBehavior = if (isBrake) ZeroPowerBehavior.BRAKE else ZeroPowerBehavior.FLOAT
    reset()
}

fun DcMotorEx.reset() {
    mode = RunMode.STOP_AND_RESET_ENCODER
    mode = RunMode.RUN_WITHOUT_ENCODER
}

fun Servo.init(isForward: Boolean = true) {
    direction = if (isForward) Servo.Direction.FORWARD else Servo.Direction.REVERSE
}