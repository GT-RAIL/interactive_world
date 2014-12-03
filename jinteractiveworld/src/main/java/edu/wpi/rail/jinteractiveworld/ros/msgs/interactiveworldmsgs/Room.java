package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonObject;
import java.io.StringReader;
import java.util.Arrays;

/**
 * The interactive_world_msgs/Room message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 2, 2014
 */
public class Room extends Message {

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
	 * The name of the POI field for the message.
	 */
	public static final String FIELD_SURFACES = "surfaces";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/Room";

	private final String name;
	private final double width, height;
	private final Pose pose;
	private final Surface[] surfaces;

	/**
	 * Create a new empty Room message.
	 */
	public Room() {
		this("", 0, 0, new Pose(), new Surface[0]);
	}

	/**
	 * Create a new Room message based on the given information.
	 *
	 * @param name
	 *            The name of the Room.
	 * @param width
	 *            The width of the Room.
	 * @param height
	 *            The height of the Room.
	 * @param pose
	 *            The pose of the Room.
	 * @param surfaces
	 *            The surfaces of the Room.
	 */
	public Room(String name, double width, double height, Pose pose, Surface[] surfaces) {
		// build the JSON object
		super(Json.createObjectBuilder().add(Room.FIELD_NAME, name)
				.add(Room.FIELD_WIDTH, width)
				.add(Room.FIELD_HEIGHT, height)
				.add(Room.FIELD_POSE, pose.toJsonObject())
				.add(Room.FIELD_SURFACES, Json.createReader(new StringReader(Arrays.deepToString(surfaces))).readArray()).build(), Room.TYPE);
		this.name = name;
		this.width = width;
		this.height = height;
		this.pose = pose;
		this.surfaces = surfaces;
	}

	/**
	 * Get the name value of this Room.
	 *
	 * @return The name value of this Room.
	 */
	public String getName() {
		return this.name;
	}

	/**
	 * Get the width value of this Room.
	 *
	 * @return The width value of this Room.
	 */
	public double getWidth() {
		return this.width;
	}

	/**
	 * Get the height value of this Room.
	 *
	 * @return The height value of this Room.
	 */
	public double getHeight() {
		return this.height;
	}

	/**
	 * Get the pose value of this Room.
	 *
	 * @return The pose value of this Room.
	 */
	public Pose getPose() {
		return this.pose;
	}

	/**
	 * Get the surfaces value of this Room.
	 *
	 * @return The surfaces value of this Room.
	 */
	public Surface[] getSurfaces() {
		return this.surfaces;
	}

	/**
	 * Create a clone of this Placement.
	 */
	@Override
	public Room clone() {
		return new Room(this.name, this.width, this.height, this.pose, this.surfaces);
	}

	/**
	 * Create a new Placement based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Point message based on the given JSON string.
	 */
	public static Room fromJsonString(String jsonString) {
		// convert to a message
		return Room.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new Placement based on the given Message. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Point message based on the given Message.
	 */
	public static Room fromMessage(Message m) {
		// get it from the JSON object
		return Room.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new Placement based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static Room fromJsonObject(JsonObject jsonObject) {
		// check the fields
		String name = jsonObject.containsKey(Room.FIELD_NAME) ? jsonObject
				.getString(Room.FIELD_NAME) : "";
		double width = jsonObject.containsKey(Room.FIELD_WIDTH) ? jsonObject
				.getJsonNumber(Room.FIELD_WIDTH).doubleValue() : 0.0;
		double height = jsonObject.containsKey(Room.FIELD_HEIGHT) ? jsonObject
				.getJsonNumber(Room.FIELD_HEIGHT).doubleValue() : 0.0;
		Pose pose = jsonObject.containsKey(Room.FIELD_POSE) ? Pose
				.fromJsonObject(jsonObject.getJsonObject(Room.FIELD_POSE))
				: new Pose();
		JsonArray jsonSurfaces = jsonObject.getJsonArray(Room.FIELD_SURFACES);
		if(jsonSurfaces == null) {
			return new Room(name, width, height, pose, new Surface[0]);
		} else {
			Surface[] surfaces = new Surface[jsonSurfaces.size()];
			for(int i = 0; i < surfaces.length; ++i) {
				surfaces[i] = Surface.fromJsonObject(jsonSurfaces.getJsonObject(i));
			}

			return new Room(name, width, height, pose, surfaces);
		}
	}
}
