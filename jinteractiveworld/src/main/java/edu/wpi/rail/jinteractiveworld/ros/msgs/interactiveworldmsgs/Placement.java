package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import javax.json.Json;
import javax.json.JsonObject;

import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;
import edu.wpi.rail.jrosbridge.messages.geometry.Quaternion;

/**
 * The interactive_world_msgs/Placement message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version August 14, 2014
 */
public class Placement extends Message {

	/**
	 * The name of the object field for the message.
	 */
	public static final String FIELD_OBJ = "obj";

	/**
	 * The name of the pose field for the message.
	 */
	public static final String FIELD_POSE = "pose";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/Placement";

	private final Object obj;
	private final Pose pose;

	/**
	 * Create a new empty Placement message.
	 */
	public Placement() {
		this(new Object(), new Pose());
	}

	/**
	 * Create a new Placement message based on the given information.
	 * 
	 * @param obj
	 *            The object in the placement.
	 * @param pose
	 *            The pose of the object.
	 */
	public Placement(Object obj, Pose pose) {
		// build the JSON object
		super(Json.createObjectBuilder()
				.add(Placement.FIELD_OBJ, obj.toJsonObject())
				.add(Placement.FIELD_POSE, pose.toJsonObject()).build(),
				Placement.TYPE);
		this.obj = obj;
		this.pose = pose;
	}

	/**
	 * Create a Placement message based on the JInteractiveWorld placement.
	 * 
	 * @param p
	 *            The JInteractiveWorld to create a placement from.
	 */
	public Placement(edu.wpi.rail.jinteractiveworld.model.Placement p) {
		this(new Object(p.getObject()), new Pose(new Point(p.getTransform()
				.getVector3().getX(), p.getTransform().getVector3().getY(), p
				.getTransform().getVector3().getZ()), new Quaternion(p
				.getTransform().getRotationMatrix().toVector4().getX(), p
				.getTransform().getRotationMatrix().toVector4().getY(), p
				.getTransform().getRotationMatrix().toVector4().getZ(), p
				.getTransform().getRotationMatrix().toVector4().getW())));
	}

	/**
	 * Get the object value of this object.
	 * 
	 * @return The object value of this object.
	 */
	public Object getObj() {
		return this.obj;
	}

	/**
	 * Get the pose value of this object.
	 * 
	 * @return The pose value of this object.
	 */
	public Pose getPose() {
		return this.pose;
	}

	/**
	 * Create a clone of this Placement.
	 */
	@Override
	public Placement clone() {
		return new Placement(this.obj, this.pose);
	}

	/**
	 * Create a new Placement based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Point message based on the given JSON string.
	 */
	public static Placement fromJsonString(String jsonString) {
		// convert to a message
		return Placement.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new Placement based on the given Message. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Point message based on the given Message.
	 */
	public static Placement fromMessage(Message m) {
		// get it from the JSON object
		return Placement.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new Placement based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonPlacement
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static Placement fromJsonObject(JsonObject jsonObject) {
		// check the fields
		Object obj = jsonObject.containsKey(Placement.FIELD_OBJ) ? Object
				.fromJsonObject(jsonObject.getJsonObject(Placement.FIELD_OBJ))
				: new Object();
		Pose pose = jsonObject.containsKey(Placement.FIELD_POSE) ? Pose
				.fromJsonObject(jsonObject.getJsonObject(Placement.FIELD_POSE))
				: new Pose();
		return new Placement(obj, pose);
	}
}
