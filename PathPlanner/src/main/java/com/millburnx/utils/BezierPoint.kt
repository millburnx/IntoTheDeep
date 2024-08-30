package com.millburnx.utils

import com.millburnx.utils.vec2d.Vec2d

data class BezierPoint(var anchor: Vec2d, var prev: Vec2d? = null, var next: Vec2d? = null)