package com.millburnx.dashboard.canvas.canvasOp

abstract class CanvasOp(val type: Type) {
    enum class Type {
        ALPHA,
        CIRCLE,
        FILL,
        GRID,
        IMAGE,
        POLYGON,
        POLYLINE,
        ROTATION,
        SCALE,
        STROKE,
        STROKE_WIDTH,
        TEXT,
        TRANSLATE,
    }
}

