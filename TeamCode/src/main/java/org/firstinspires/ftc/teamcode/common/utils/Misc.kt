package org.firstinspires.ftc.teamcode.common.utils

import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.acmerobotics.roadrunner.geometry.Pose2d as RRVec2d

fun DcMotorEx.init(isForward: Boolean = true, isBrake: Boolean = false) {
    direction = if (isForward) Direction.FORWARD else Direction.REVERSE
    zeroPowerBehavior = if (isBrake) ZeroPowerBehavior.BRAKE else ZeroPowerBehavior.FLOAT
    reset()
}

fun DcMotorEx.reset() {
    mode = RunMode.STOP_AND_RESET_ENCODER
    mode = RunMode.RUN_WITHOUT_ENCODER
}

fun RRVec2d.toVec2d(): Vec2d = Vec2d(x, y)
fun RRVec2d.toCorrectedVec2d(): Vec2d = Vec2d.fromRR(this.toVec2d())