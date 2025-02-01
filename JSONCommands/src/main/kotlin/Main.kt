package com.millburnx.jsoncommands

import com.google.gson.GsonBuilder
import kotlin.reflect.KClass

public class Parser(
    classes: List<KClass<out BaseObject>>,
) {
    public constructor(vararg classes: KClass<out BaseObject>) : this(classes.toList())

    private val gson =
        GsonBuilder()
            .registerTypeAdapter(BaseObject::class.java, Deserializer(classes))
            .create()

    public fun parse(json: String): BaseObject = gson.fromJson(json, BaseObject::class.java)
}

public fun main() {
    val parser =
        Parser(
            SequentialCommandGroup::class,
            ParallelCommandGroup::class,
            ParallelCommandGroup::class,
            ParallelDeadlineGroup::class,
        )

    val json =
        "{ \"type\": \"CommandGroup/SequentialCommandGroup\", \"commands\": [{ \"type\": \"CommandGroup/ParallelCommandGroup\", \"commands\": []}] }"
    println(json)

    val shape1 = parser.parse(json)
    println(shape1)
}
