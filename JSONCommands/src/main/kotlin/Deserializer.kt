package com.millburnx.jsoncommands

import com.google.gson.JsonDeserializationContext
import com.google.gson.JsonDeserializer
import com.google.gson.JsonElement
import com.google.gson.JsonObject
import com.google.gson.JsonParseException
import java.lang.reflect.Type
import kotlin.reflect.KClass
import kotlin.reflect.full.memberProperties
import kotlin.reflect.full.primaryConstructor
import kotlin.reflect.javaType

public class Deserializer<baseClass : BaseObject>(
    public val classes: List<KClass<out baseClass>>,
) : JsonDeserializer<baseClass> {
    public val classesMap: Map<String, KClass<out baseClass>> =
        classes.associateBy {
            it.java
                .getDeclaredConstructor()
                .newInstance()
                .type
        }

    @OptIn(ExperimentalStdlibApi::class)
    override fun deserialize(
        json: JsonElement,
        typeOfT: Type,
        context: JsonDeserializationContext,
    ): baseClass {
        val jsonObject = json.asJsonObject
        val type = jsonObject["type"].asString // Extract the type field

        val targetClass = classesMap[type] ?: throw JsonParseException("Unknown type: $type")

        // type check
        if (!isValid(jsonObject, targetClass)) {
            throw JsonParseException("Invalid fields for type: $type")
        }
//        return context.deserialize(jsonObject, targetClass.java)
        val constructor = targetClass.primaryConstructor!!
        val instance =
            constructor.callBy(
                constructor.parameters.associateWith { parameter ->
                    val name = parameter.name!!
                    val value = jsonObject[name]
                    context.deserialize(value, parameter.type.javaType)
                },
            )
        return instance
    }

    private fun isValid(
        jsonObject: JsonObject,
        targetClass: KClass<out baseClass>,
    ): Boolean {
        val fields = jsonObject.entrySet().map { it.key }
        val expectedFields = targetClass.memberProperties.map { it.name }
        var nonNullExpectedFields =
            targetClass.memberProperties.filter { it.returnType.isMarkedNullable.not() }.map { it.name }
        val containsRequired = nonNullExpectedFields.all { fields.contains(it) } // if its nullable, it can be missing
        val containsExtra = fields.any { !expectedFields.contains(it) }
        return containsRequired && !containsExtra
    }
}
