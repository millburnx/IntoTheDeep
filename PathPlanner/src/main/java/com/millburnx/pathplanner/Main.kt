package com.millburnx.pathplanner

import java.awt.BorderLayout
import java.awt.Color
import java.awt.Dimension
import java.awt.Rectangle
import javax.swing.JFrame
import javax.swing.JLayeredPane
import javax.swing.JPanel

fun main(args: Array<String>) {
    val frame = getJFrame()
    val panel1 = JPanel()
    panel1.background = Color.RED
    val panel2 = JPanel()
    panel2.background = Color.GREEN
    val panel3 = JPanel()
    panel3.background = Color.BLUE
    val wrapper = JPanel()
    wrapper.layout = BorderLayout()
    println(wrapper.layout)
    wrapper.add(panel1, BorderLayout.NORTH)
    wrapper.add(panel2, BorderLayout.CENTER)
    wrapper.add(panel3, BorderLayout.SOUTH)
    val popoverLayer = object : JPanel() {
        override fun paintComponent(g: java.awt.Graphics) {
            g.color = Color(0f, 0f, 0f, 0.5f)
            g.fillRect(0, 0, width, height)
            super.paintComponent(g)
        }
    }
    popoverLayer.isOpaque = false
    val wrapper2 = JLayeredPane()
    frame.add(wrapper2)
    wrapper.bounds = Rectangle(0, 0, frame.width, frame.height)
    popoverLayer.bounds = Rectangle(0, 0, frame.width, frame.height)
    wrapper2.add(wrapper, JLayeredPane.PALETTE_LAYER, 1)
    wrapper2.add(popoverLayer, JLayeredPane.MODAL_LAYER, 0)
    wrapper2.background = Color(0, 0, 0, 0)
    println("${wrapper2.getLayer(wrapper)}")
    println("${wrapper2.getLayer(popoverLayer)}")
}

fun getJFrame(): JFrame {
    val frame = JFrame("Pure Pursuit")
    frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    frame.size = Dimension(800, 600)
    frame.setLocationRelativeTo(null)
    frame.isVisible = true
    return frame
}