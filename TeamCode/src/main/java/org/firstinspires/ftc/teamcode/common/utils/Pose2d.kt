package org.firstinspires.ftc.teamcode.common.utils

import com.millburnx.utils.Vec2d
import kotlin.math.absoluteValue

data class Pose2d(val position: Vec2d = Vec2d(), val heading: Double = 0.0) {
    constructor(x: Double, y: Double, heading: Double) : this(Vec2d(x, y), heading)
    constructor(position: Double, heading: Double) : this(Vec2d(position), heading)

    val x: Double
        get() = position.x
    val y: Double
        get() = position.y
    val degrees: Double
        get() = heading
    val radians: Double
        get() = Math.toRadians(heading)
    val rotation: Double
        get() = heading

    companion object {
        fun fromRadians(position: Vec2d, radians: Double): Pose2d {
            return Pose2d(position, Math.toDegrees(radians))
        }

        fun fromRadians(x: Double, y: Double, radians: Double): Pose2d {
            return Pose2d(Vec2d(x, y), Math.toDegrees(radians))
        }

        fun fromRR(pose: com.acmerobotics.roadrunner.geometry.Pose2d): Pose2d {
            return Pose2d(pose.x, pose.y, pose.heading)
        }
    }

    fun toRR(): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(position.x, position.y, heading)
    }

    operator fun unaryMinus(): Pose2d = Pose2d(-position, -heading)
    fun flipPos(): Pose2d = Pose2d(-position, heading)
    fun abs(): Pose2d = Pose2d(position.abs(), heading.absoluteValue)
    operator fun plus(other: Pose2d): Pose2d = Pose2d(position + other.position, heading + other.heading)
    operator fun plus(other: Vec2d): Pose2d = Pose2d(position + other, heading)
    operator fun minus(other: Pose2d): Pose2d = Pose2d(position - other.position, heading - other.heading)
    operator fun minus(other: Vec2d): Pose2d = Pose2d(position - other, heading)
    operator fun times(other: Vec2d): Pose2d = Pose2d(position * other, heading)
    operator fun times(scalar: Double): Pose2d = Pose2d(position * scalar, heading)
    operator fun div(other: Vec2d): Pose2d = Pose2d(position / other, heading)
    operator fun div(scalar: Double): Pose2d = Pose2d(position / scalar, heading)
}