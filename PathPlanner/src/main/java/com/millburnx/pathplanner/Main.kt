package com.millburnx.pathplanner

import com.millburnx.pathplanner.Layout.StackLayout
import java.awt.Dimension
import javax.swing.JFrame

fun main(args: Array<String>) {
    val frame = getJFrame()
    val stackLayout = StackLayout()
    stackLayout.addTo(frame)
}

fun getJFrame(): JFrame {
    val frame = JFrame("Pure Pursuit")
    frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    frame.size = Dimension(800, 600)
    frame.setLocationRelativeTo(null)
    frame.isVisible = true
    return frame
}