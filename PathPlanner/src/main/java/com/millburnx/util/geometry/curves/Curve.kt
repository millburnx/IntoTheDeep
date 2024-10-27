package com.millburnx.util.geometry.curves

import com.millburnx.util.geometry.IVec2d

public interface Curve {
    public val points: List<IVec2d>

    public fun at(t: Double): IVec2d
}