package com.millburnx.pathPlanner.layout

import com.millburnx.util.geometry.curves.CubicBezierPoint
import com.millburnx.util.javafx.Utils
import java.awt.BorderLayout
import javax.swing.JLabel
import javax.swing.JPanel
import javax.swing.JSpinner
import javax.swing.border.EmptyBorder

public typealias Path = List<CubicBezierPoint>

public const val MAX_LAYERS: Int = 10

public class EditorWrapper : JPanel() {
    //    public val editorList: EditorList = EditorList()
//    public val editor: JPanel = JPanel().apply {
//        background = Color.MAGENTA
//        layout = BorderLayout()
//        val title = editorList.model.title
//        title.border = MatteBorder(0, 0, 1, 0, Color.WHITE)
//        add(title, BorderLayout.NORTH)
//        add(editorList, BorderLayout.CENTER)
//    }
//    public val scrollPane: JScrollPane = JScrollPane(editor).apply {
//        isOpaque = true
//    }
    public var currentLayer: Int = 0
    public var currentKnotParameter: Double = 0.0
    public var layers: Array<Path> = Array(MAX_LAYERS) { listOf<CubicBezierPoint>() }

    public val settings: JPanel = JPanel().apply {
        val layerLabel = JLabel("Layer")
        val alphaLabel = JLabel("Knot Parameter")
        val layerSelector = Utils.jNumber(0.0..MAX_LAYERS.toDouble())
        val alphaSelector = Utils.jNumber(0.0..1.0, 0.01)
        (alphaSelector.editor as JSpinner.NumberEditor).textField.columns = 3
        layerSelector.addChangeListener { _ ->
            currentLayer = (layerSelector.value as Number).toInt()
            println("Layer: $currentLayer")
        }
        alphaSelector.addChangeListener { _ ->
            currentKnotParameter = (alphaSelector.value as Number).toDouble()
            println("Knot Parameter: $currentKnotParameter")
        }
        add(layerLabel)
        add(layerSelector)
        add(alphaLabel)
        add(alphaSelector)
        revalidate()
        repaint()
    }

    init {
        layout = BorderLayout()
        border = EmptyBorder(16, 24, 16, 24)
        add(settings, BorderLayout.NORTH)
//        add(scrollPane, BorderLayout.CENTER)
    }
}