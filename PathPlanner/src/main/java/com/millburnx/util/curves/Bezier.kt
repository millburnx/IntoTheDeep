package com.millburnx.util.curves

import com.millburnx.util.IVec2d
import com.millburnx.util.Math
import com.millburnx.util.Vec2d
import com.millburnx.util.Vector2d

public abstract class Bezier : Curve {
    public abstract override val points: List<Vec2d>

    public override fun at(t: Double): Vec2d {
        // De Casteljau's algorithm
        var input = points
        var output: Array<Vec2d>? = null

        while (input.size > 1) {
            output = Array(input.size - 1) { Vector2d() }
            input.zipWithNext().forEachIndexed { index, (a, b) ->
                output[index] = Vector2d(
                    Math.lerp(a.x, b.x, t),
                    Math.lerp(a.y, b.y, t)
                )
            }
            input = output.toList()
        }

        return input[0]
    }
}

public data class LinearBezier(public val start: Vec2d, public val end: Vec2d) : Bezier() {
    public constructor(start: IVec2d, end: IVec2d) : this(Vec2d(start), Vec2d(end))

    override val points: List<Vec2d> = listOf(start, end)
}

public data class QuadraticBezier(public val start: Vec2d, public val control: Vec2d, public val end: Vec2d) :
    Bezier() {
    public constructor(start: IVec2d, control: IVec2d, end: IVec2d) : this(Vec2d(start), Vec2d(control), Vec2d(end))

    override val points: List<Vec2d> = listOf(start, control, end)
}

public data class CubicBezier(
    public val start: Vec2d,
    public val control1: Vec2d,
    public val control2: Vec2d,
    public val end: Vec2d
) : Bezier() {
    public constructor(start: IVec2d, control1: IVec2d, control2: IVec2d, end: IVec2d) : this(
        Vector2d(start),
        Vector2d(control1),
        Vector2d(control2),
        Vector2d(end)
    )

    override val points: List<Vec2d> = listOf(start, control1, control2, end)
}
