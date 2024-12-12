package com.millburnx.util.geometry.basic

import com.millburnx.util.geometry.basic.vector.Vec2d
import java.io.Serializable

public data class BBox(public val min: Vec2d, public val max: Vec2d) : Serializable {
    public val center: Vec2d
        get() = min + (size / 2)
    public val size: Vec2d
        get() = max - min
    public val width: Double
        get() = size.x
    public val height: Double
        get() = size.y
    public val topLeft: Vec2d
        get() = min
    public val topRight: Vec2d
        get() = Vec2d(max.x, min.y)
    public val bottomLeft: Vec2d
        get() = Vec2d(min.x, max.y)
    public val bottomRight: Vec2d
        get() = max

    public fun contains(point: Vec2d): Boolean {
        return point.x in min.x..max.x && point.y in min.y..max.y
    }

    public companion object {
        public fun fromPoints(points: List<Vec2d>): BBox {
            val minX = points.minOf { it.x }
            val maxX = points.maxOf { it.x }
            val minY = points.minOf { it.y }
            val maxY = points.maxOf { it.y }
            val min = Vec2d(minX, minY)
            val max = Vec2d(maxX, maxY)
            return BBox(min + (max - min) / 2, max - min)
        }
    }
}