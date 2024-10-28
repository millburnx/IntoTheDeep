package com.millburnx.util.geometry.curves

import com.millburnx.util.geometry.Vec2d

public enum class CubicBezierPointType { MAIN, ENTER, EXIT }

// used to represent a point and its handles in a cubic bezier
// (could also be used to store a multi-directional hermite)
public data class CubicBezierPoint(val point: Vec2d, val enter: Vec2d? = null, val exit: Vec2d? = null) {
    public fun getType(type: CubicBezierPointType): Vec2d? = when (type) {
        CubicBezierPointType.MAIN -> point
        CubicBezierPointType.ENTER -> enter
        CubicBezierPointType.EXIT -> exit
    }

    public fun setType(type: CubicBezierPointType, value: Vec2d?): CubicBezierPoint {
        return when (type) {
            CubicBezierPointType.MAIN -> copy(point = value!!)
            CubicBezierPointType.ENTER -> copy(enter = value)
            CubicBezierPointType.EXIT -> copy(exit = value)
        }
    }

    // same angle, same dist
    public fun mirrorMove(type: CubicBezierPointType, newPos: Vec2d): CubicBezierPoint {
        if (type == CubicBezierPointType.MAIN) {
            val diff = newPos - point
            return copy(point = newPos, exit = exit?.plus(diff), enter = enter?.plus(diff))
        }
        val centerDiff = (point - newPos) * if (type == CubicBezierPointType.ENTER) 1 else -1
        return copy(point = point, exit = point.plus(centerDiff), enter = point.minus(centerDiff))
    }

    // same angle, diff dist
    public fun fixedMove(type: CubicBezierPointType, newPos: Vec2d): CubicBezierPoint {
        if (type == CubicBezierPointType.MAIN) {
            val diff = newPos - point
4            return copy(point = newPos, exit = exit?.plus(diff), enter = enter?.plus(diff))
        }
    }
}