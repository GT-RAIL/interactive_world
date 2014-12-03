package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;
import edu.wpi.rail.jrosbridge.messages.geometry.Quaternion;

import javax.json.Json;
import javax.json.JsonObject;

/**
 * The interactive_world_msgs/PointOfInterest message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 2, 2014
 */
public class PointOfInterest extends Message {

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
	 * The name of the pose field for the message.
	 */
	public static final String FIELD_POSE = "pose";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/PointOfInterest";

	private final String name;
	private final double width, height;
	private final Pose pose;

	/**
	 * Create a new empty POI message.
	 */
	public PointOfInterest() {
		this("", 0, 0, new Pose());
	}

	/**
	 * Create a new POI message based on the given information.
	 *
	 * @param name
	 *            The name of the POI.
	 * @param width
	 *            The width of the POI.
	 * @param height
	 *            The height of the POI.
	 * @param pose
	 *            The pose of the POI.
	 */
	public PointOfInterest(String name, double width, double height, Pose pose) {
		// build the JSON object
		super(Json.createObjectBuilder().add(PointOfInterest.FIELD_NAME, name)
				.add(PointOfInterest.FIELD_WIDTH, width)
				.add(PointOfInterest.FIELD_HEIGHT, height)
				.add(PointOfInterest.FIELD_POSE, pose.toJsonObject()).build(), PointOfInterest.TYPE);
		this.name = name;
		this.width = width;
		this.height = height;
		this.pose = pose;
	}

	/**
	 * Get the name value of this POI.
	 *
	 * @return The name value of this POI.
	 */
	public String getName() {
		return this.name;
	}

	/**
	 * Get the width value of this POI.
	 *
	 * @return The width value of this POI.
	 */
	public double getWidth() {
		return this.width;
	}

	/**
	 * Get the height value of this POI.
	 *
	 * @return The height value of this POI.
	 */
	public double getHeight() {
		return this.height;
	}

	/**
	 * Get the pose value of this POI.
	 * 
	 * @return The pose value of this POI.
	 */
	public Pose getPose() {
		return this.pose;
	}

	/**
	 * Create a clone of this Placement.
	 */
	@Override
	public PointOfInterest clone() {
		return new PointOfInterest(this.name, this.width, this.height, this.pose);
	}

	/**
	 * Create a new Placement based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Point message based on the given JSON string.
	 */
	public static PointOfInterest fromJsonString(String jsonString) {
		// convert to a message
		return PointOfInterest.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new Placement based on the given Message. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Point message based on the given Message.
	 */
	public static PointOfInterest fromMessage(Message m) {
		// get it from the JSON object
		return PointOfInterest.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new Placement based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static PointOfInterest fromJsonObject(JsonObject jsonObject) {
		// check the fields
		String name = jsonObject.containsKey(PointOfInterest.FIELD_NAME) ? jsonObject
				.getString(PointOfInterest.FIELD_NAME) : "";
		double width = jsonObject.containsKey(PointOfInterest.FIELD_WIDTH) ? jsonObject
				.getJsonNumber(PointOfInterest.FIELD_WIDTH).doubleValue() : 0.0;
		double height = jsonObject.containsKey(PointOfInterest.FIELD_HEIGHT) ? jsonObject
				.getJsonNumber(PointOfInterest.FIELD_HEIGHT).doubleValue() : 0.0;
		Pose pose = jsonObject.containsKey(PointOfInterest.FIELD_POSE) ? Pose
				.fromJsonObject(jsonObject.getJsonObject(PointOfInterest.FIELD_POSE))
				: new Pose();
		return new PointOfInterest(name, width, height, pose);
	}
}
