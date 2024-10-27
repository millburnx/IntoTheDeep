package com.millburnx.util.curves

import com.millburnx.util.IVec2d
import com.millburnx.util.Vec2d

public data class Hermite(public val p0: Vec2d, public val v0: Vec2d, public val p1: Vec2d, public val v1: Vec2d) :
    Curve {
    public constructor(p0: IVec2d, v0: IVec2d, p1: IVec2d, v1: IVec2d) : this(
        Vec2d(p0),
        Vec2d(v0),
        Vec2d(p1),
        Vec2d(v1)
    )

    override val points: List<Vec2d> = listOf(p0, p1)

    @Suppress("detekt:MagicNumber")
    public val bezier: CubicBezier = CubicBezier(p0, p0 + v0 / 3, p1 - v1 / 3, p1)

    override fun at(t: Double): Vec2d {
        return bezier.at(t)
    }
}
