package com.millburnx.pathplanner.Layout

import java.awt.BorderLayout
import java.awt.Color
import javax.swing.JPanel

class MainLayout : JPanel() {
    val navbar = JPanel().apply {
        background = Color.RED
    }
    val viewer = JPanel().apply {
        background = Color.GREEN
    }
    val editor = JPanel().apply {
        background = Color.BLUE
    }

    init {
        layout = BorderLayout()
        add(navbar, BorderLayout.NORTH)
        add(viewer, BorderLayout.CENTER)
        add(editor, BorderLayout.SOUTH)
    }
}