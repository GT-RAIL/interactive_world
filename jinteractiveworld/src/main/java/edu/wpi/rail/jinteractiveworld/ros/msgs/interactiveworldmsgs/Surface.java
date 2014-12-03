package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonObject;
import java.io.StringReader;
import java.util.Arrays;

/**
 * The interactive_world_msgs/Surface message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 2, 2014
 */
public class Surface extends Message {

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
	public static final String FIELD_POIS = "pois";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/Surface";

	private final String name;
	private final double width, height;
	private final Pose pose;
	private final PointOfInterest[] pois;

	/**
	 * Create a new empty POI message.
	 */
	public Surface() {
		this("", 0, 0, new Pose(), new PointOfInterest[0]);
	}

	/**
	 * Create a new Surface message based on the given information.
	 *
	 * @param name
	 *            The name of the Surface.
	 * @param width
	 *            The width of the Surface.
	 * @param height
	 *            The height of the Surface.
	 * @param pose
	 *            The pose of the Surface.
	 * @param pois
	 *            The POIs of the Surface.
	 */
	public Surface(String name, double width, double height, Pose pose, PointOfInterest[] pois) {
		// build the JSON object
		super(Json.createObjectBuilder().add(Surface.FIELD_NAME, name)
				.add(Surface.FIELD_WIDTH, width)
				.add(Surface.FIELD_HEIGHT, height)
				.add(Surface.FIELD_POSE, pose.toJsonObject())
				.add(Surface.FIELD_POIS, Json.createReader(new StringReader(Arrays.deepToString(pois))).readArray()).build(), Surface.TYPE);
		this.name = name;
		this.width = width;
		this.height = height;
		this.pose = pose;
		this.pois = pois;
	}

	/**
	 * Get the name value of this Surface.
	 *
	 * @return The name value of this Surface.
	 */
	public String getName() {
		return this.name;
	}

	/**
	 * Get the width value of this Surface.
	 *
	 * @return The width value of this Surface.
	 */
	public double getWidth() {
		return this.width;
	}

	/**
	 * Get the height value of this Surface.
	 *
	 * @return The height value of this Surface.
	 */
	public double getHeight() {
		return this.height;
	}

	/**
	 * Get the pose value of this Surface.
	 *
	 * @return The pose value of this Surface.
	 */
	public Pose getPose() {
		return this.pose;
	}

	/**
	 * Get the POIs value of this Surface.
	 *
	 * @return The POIs value of this Surface.
	 */
	public PointOfInterest[] getPOIs() {
		return this.pois;
	}

	/**
	 * Create a clone of this Placement.
	 */
	@Override
	public Surface clone() {
		return new Surface(this.name, this.width, this.height, this.pose, this.pois);
	}

	/**
	 * Create a new Placement based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Point message based on the given JSON string.
	 */
	public static Surface fromJsonString(String jsonString) {
		// convert to a message
		return Surface.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new Placement based on the given Message. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Point message based on the given Message.
	 */
	public static Surface fromMessage(Message m) {
		// get it from the JSON object
		return Surface.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new Placement based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static Surface fromJsonObject(JsonObject jsonObject) {
		// check the fields
		String name = jsonObject.containsKey(Surface.FIELD_NAME) ? jsonObject
				.getString(Surface.FIELD_NAME) : "";
		double width = jsonObject.containsKey(Surface.FIELD_WIDTH) ? jsonObject
				.getJsonNumber(Surface.FIELD_WIDTH).doubleValue() : 0.0;
		double height = jsonObject.containsKey(Surface.FIELD_HEIGHT) ? jsonObject
				.getJsonNumber(Surface.FIELD_HEIGHT).doubleValue() : 0.0;
		Pose pose = jsonObject.containsKey(Surface.FIELD_POSE) ? Pose
				.fromJsonObject(jsonObject.getJsonObject(Surface.FIELD_POSE))
				: new Pose();
		JsonArray jsonPois = jsonObject.getJsonArray(Surface.FIELD_POIS);
		if(jsonPois == null) {
			return new Surface(name, width, height, pose, new PointOfInterest[0]);
		} else {
			PointOfInterest[] pois = new PointOfInterest[jsonPois.size()];
			for(int i = 0; i < pois.length; ++i) {
				pois[i] = PointOfInterest.fromJsonObject(jsonPois.getJsonObject(i));
			}

			return new Surface(name, width, height, pose, pois);
		}
	}
}
