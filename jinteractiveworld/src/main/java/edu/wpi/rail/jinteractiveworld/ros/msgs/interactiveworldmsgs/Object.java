package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import javax.json.Json;
import javax.json.JsonObject;

import edu.wpi.rail.jrosbridge.messages.Message;

/**
 * The interactive_world_msgs/Object message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version August 14, 2014
 */
public class Object extends Message {

	/**
	 * The name of the name field for the message.
	 */
	public static final String FIELD_NAME = "name";

	/**
	 * The name of the width field for the message.
	 */
	public static final String FIELD_WIDTH = "width";

	/**
	 * The name of the height field for the message.
	 */
	public static final String FIELD_HEIGHT = "height";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/Object";

	private final String name;
	private final double width, height;

	/**
	 * Create a new empty Object message.
	 */
	public Object() {
		this("", 0, 0);
	}

	/**
	 * Create a new Object message with the given information.
	 * 
	 * @param name
	 *            The name of the object.
	 * @param width
	 *            The width of the object.
	 * @param height
	 *            The height of the object.
	 */
	public Object(String name, double width, double height) {
		// build the JSON object
		super(Json.createObjectBuilder().add(Object.FIELD_NAME, name)
				.add(Object.FIELD_WIDTH, width)
				.add(Object.FIELD_HEIGHT, height).build(), Object.TYPE);
		this.name = name;
		this.width = width;
		this.height = height;
	}

	/**
	 * Create a new Object message from a JInteractiveWorld object.
	 * 
	 * @param object
	 *            The JInteractiveWorld to create a message from.
	 */
	public Object(edu.wpi.rail.jinteractiveworld.world.Object object) {
		this(object.getName(), object.getWidth(), object.getHeight());
	}

	/**
	 * Get the name value of this object.
	 * 
	 * @return The name value of this object.
	 */
	public String getName() {
		return this.name;
	}

	/**
	 * Get the width value of this object.
	 * 
	 * @return The width value of this object.
	 */
	public double getWidth() {
		return this.width;
	}

	/**
	 * Get the height value of this object.
	 * 
	 * @return The height value of this object.
	 */
	public double getHeight() {
		return this.height;
	}

	/**
	 * Create a clone of this Object.
	 */
	@Override
	public Object clone() {
		return new Object(this.name, this.width, this.height);
	}

	/**
	 * Create a new Object based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Point message based on the given JSON string.
	 */
	public static Object fromJsonString(String jsonString) {
		// convert to a message
		return Object.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new Object based on the given Message. Any missing values will
	 * be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Point message based on the given Message.
	 */
	public static Object fromMessage(Message m) {
		// get it from the JSON object
		return Object.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new Object based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static Object fromJsonObject(JsonObject jsonObject) {
		// check the fields
		String name = jsonObject.containsKey(Object.FIELD_NAME) ? jsonObject
				.getString(Object.FIELD_NAME) : "";
		double width = jsonObject.containsKey(Object.FIELD_WIDTH) ? jsonObject
				.getJsonNumber(Object.FIELD_WIDTH).doubleValue() : 0.0;
		double height = jsonObject.containsKey(Object.FIELD_HEIGHT) ? jsonObject
				.getJsonNumber(Object.FIELD_HEIGHT).doubleValue() : 0.0;
		return new Object(name, width, height);
	}
}
