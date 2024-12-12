package com.millburnx.util.geometry.basic

import com.millburnx.util.geometry.basic.vector.Vec2
import com.millburnx.util.geometry.basic.vector.Vec2d
import java.io.Serializable

public data class Circle(public val center: Vec2d, public val radius: Double) : Serializable {
    public fun contains(point: Vec2d): Boolean {
        return center.distance(point) <= radius
    }

    public operator fun plus(vec: Vec2<*, *>): Circle = Circle(center + Vec2d(vec), radius)
    public operator fun minus(vec: Vec2<*, *>): Circle = Circle(center - Vec2d(vec), radius)
    public operator fun times(scalar: Number): Circle = Circle(center, radius * scalar.toDouble())
    public operator fun div(scalar: Number): Circle = Circle(center, radius / scalar.toDouble())
    public operator fun unaryMinus(): Circle = Circle(-center, radius)
}