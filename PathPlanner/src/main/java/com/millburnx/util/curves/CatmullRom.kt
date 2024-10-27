package com.millburnx.util.curves

import com.millburnx.util.IVec2d
import com.millburnx.util.Vec2d
import kotlin.math.pow

public data class CatmullRom(
    public val p0: Vec2d,
    public val p1: Vec2d,
    public val p2: Vec2d,
    public val p3: Vec2d,
    public val alpha: Double = 0.5
) : Curve {
    public constructor(p0: IVec2d, p1: IVec2d, p2: IVec2d, p3: IVec2d, alpha: Double = 0.5) : this(
        Vec2d(p0),
        Vec2d(p1),
        Vec2d(p2),
        Vec2d(p3),
        alpha
    )

    override val points: List<Vec2d> = listOf(p0, p1, p2, p3)

    public val hermite: Hermite = run {
        val t0 = 0
        val t1 = p0.distanceTo(p1).pow(alpha) + t0
        val t2 = p1.distanceTo(p2).pow(alpha) + t1
        val t3 = p2.distanceTo(p3).pow(alpha) + t2

        val c1 = (t2 - t1) / (t2 - t0)
        val c2 = (t1 - t0) / (t2 - t0)
        val d1 = (t3 - t2) / (t3 - t1)
        val d2 = (t2 - t1) / (t3 - t1)

        val v1 = ((p1 - p0) * c1 / (t1 - t0) + (p2 - p1) * c2 / (t2 - t1)) * (t2 - t1) // velocity/derivative at p1
        val v2 = ((p2 - p1) * d1 / (t2 - t1) + (p3 - p2) * d2 / (t3 - t2)) * (t2 - t1) // velocity/derivative at p2
        Hermite(p1, p2, v1, v2)
    }

    public val v1: Vec2d
        get() = hermite.v0

    public val v2: Vec2d
        get() = hermite.v1

    override fun at(t: Double): Vec2d {
        return hermite.at(t)
    }
}
