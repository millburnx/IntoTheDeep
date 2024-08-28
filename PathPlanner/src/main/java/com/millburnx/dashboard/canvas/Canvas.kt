package com.millburnx.dashboard.canvas

import com.millburnx.dashboard.canvas.canvasOp.CanvasOp

// https://acmerobotics.github.io/ftc-dashboard/javadoc/com/acmerobotics/dashboard/canvas/Canvas.html
interface Canvas {
    fun getOperations(): List<CanvasOp>
    fun clear()
    fun drawGrid(
        x: Double,
        y: Double,
        width: Double,
        height: Double,
        numTicksX: Int,
        numTicksY: Int,
        theta: Double = 0.0,
        pivotX: Double = 0.0,
        pivotY: Double = 0.0,
        usePageFrame: Boolean = true,
    ): Canvas

    fun drawImage(
        path: String,
        x: Double,
        y: Double,
        width: Double,
        height: Double,
        theta: Double = 0.0,
        pivotX: Double = 0.0,
        pivotY: Double = 0.0,
        usePageFrame: Boolean = true,
    ): Canvas

    fun fillCircle(x: Double, y: Double, radius: Double): Canvas
    fun fillPolygon(xPoints: DoubleArray, yPoints: DoubleArray): Canvas
    fun fillRect(x: Double, y: Double, width: Double, height: Double): Canvas
    fun fillText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean = true,
    ): Canvas

    fun setAlpha(alpha: Double): Canvas
    fun setFill(color: String): Canvas
    fun setRotation(radians: Double): Canvas
    fun setScale(scaleX: Double, scaleY: Double): Canvas
    fun setStroke(color: String): Canvas
    fun setStrokeWidth(width: Int): Canvas
    fun setTranslation(x: Double, y: Double): Canvas
    fun strokeCircle(x: Double, y: Double, radius: Double): Canvas
    fun strokeLine(x1: Double, y1: Double, x2: Double, y2: Double): Canvas
    fun strokePolygon(xPoints: DoubleArray, yPoints: DoubleArray): Canvas
    fun strokePolyline(xPoints: DoubleArray, yPoints: DoubleArray): Canvas
    fun strokeRect(x: Double, y: Double, width: Double, height: Double): Canvas
    fun strokeText(
        text: String,
        x: Double,
        y: Double,
        font: String,
        theta: Double,
        usePageFrame: Boolean = true,
    ): Canvas
}