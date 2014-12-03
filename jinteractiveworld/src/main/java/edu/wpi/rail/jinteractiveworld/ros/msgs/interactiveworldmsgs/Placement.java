package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonObject;
import java.io.StringReader;
import java.util.Arrays;

/**
 * The interactive_world_msgs/Placement message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 2, 2014
 */
public class Placement extends Message {

	/**
	 * The name of the item field for the message.
	 */
	public static final String FIELD_ITEM = "item";

	/**
	 * The name of the room field for the message.
	 */
	public static final String FIELD_ROOM = "room";

	/**
	 * The name of the surface field for the message.
	 */
	public static final String FIELD_SURFACE = "surface";

	/**
	 * The name of the reference frame field for the message.
	 */
	public static final String FIELD_REFERENCE_FRAME_ID = "reference_frame_id";

	/**
	 * The name of the position field for the message.
	 */
	public static final String FIELD_POSITION = "position";

	/**
	 * The name of the rotation field for the message.
	 */
	public static final String FIELD_ROTATION = "rotation";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/Placement";

	private final Item item;
	private final Room room;
	private final Surface surface;
	private final String referenceFrameId;
	private final Point position;
	private final double rotation;

	/**
	 * Create a new empty Placement message.
	 */
	public Placement() {
		this(new Item(), new Room(), new Surface(), "", new Point(), 0.0);
	}

	/**
	 * Create a new Placement message based on the given information.
	 *
	 * @param item
	 *            The item of the Placement.
	 * @param room
	 *            The room of the Placement.
	 * @param surface
	 *            The surface of the Placement.
	 * @param referenceFrameId
	 *            The reference frame of the Placement.
	 * @param position
	 *            The position frame of the Placement.
	 * @param rotation
	 *            The rotation of the Placement.
	 */
	public Placement(Item item, Room room, Surface surface, String referenceFrameId, Point position, double rotation) {
		// build the JSON object
		super(Json.createObjectBuilder().add(Placement.FIELD_ITEM, item.toJsonObject())
				.add(Placement.FIELD_ROOM, room.toJsonObject())
				.add(Placement.FIELD_SURFACE, surface.toJsonObject())
				.add(Placement.FIELD_REFERENCE_FRAME_ID, referenceFrameId)
				.add(Placement.FIELD_POSITION, position.toJsonObject())
				.add(Placement.FIELD_ROTATION, rotation).build(), Placement.TYPE);
		this.item = item;
		this.room = room;
		this.surface = surface;
		this.referenceFrameId = referenceFrameId;
		this.position = position;
		this.rotation = rotation;
	}

	/**
	 * Get the item value of this Placement.
	 *
	 * @return The item value of this Placement.
	 */
	public Item getItem() {
		return this.item;
	}

	/**
	 * Get the room value of this Placement.
	 *
	 * @return The room value of this Placement.
	 */
	public Room getRoom() {
		return this.room;
	}

	/**
	 * Get the surface value of this Placement.
	 *
	 * @return The surface value of this Placement.
	 */
	public Surface getSurface() {
		return this.surface;
	}

	/**
	 * Get the reference frame value of this Placement.
	 *
	 * @return The reference frame value of this Placement.
	 */
	public String getReferenceFrameId() {
		return this.referenceFrameId;
	}

	/**
	 * Get the position value of this Placement.
	 *
	 * @return The position value of this Placement.
	 */
	public Point getPosition() {
		return this.position;
	}

	/**
	 * Get the rotation value of this Placement.
	 *
	 * @return The rotation value of this Placement.
	 */
	public double getRotation() {
		return this.rotation;
	}

	/**
	 * Create a clone of this Placement.
	 */
	@Override
	public Placement clone() {
		return new Placement(this.item, this.room, this.surface, this.referenceFrameId, this.position, this.rotation);
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
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static Placement fromJsonObject(JsonObject jsonObject) {
		// check the fields
		Item item = jsonObject.containsKey(Placement.FIELD_ITEM) ? Item
				.fromJsonObject(jsonObject.getJsonObject(Placement.FIELD_ITEM))
				: new Item();
		Room room = jsonObject.containsKey(Placement.FIELD_ROOM) ? Room
				.fromJsonObject(jsonObject.getJsonObject(Placement.FIELD_ROOM))
				: new Room();
		Surface surface = jsonObject.containsKey(Placement.FIELD_SURFACE) ? Surface
				.fromJsonObject(jsonObject.getJsonObject(Placement.FIELD_SURFACE))
				: new Surface();
		String referenceFrameId = jsonObject.containsKey(Placement.FIELD_REFERENCE_FRAME_ID) ? jsonObject
				.getString(Placement.FIELD_REFERENCE_FRAME_ID) : "";
		Point position = jsonObject.containsKey(Placement.FIELD_POSITION) ? Point
				.fromJsonObject(jsonObject.getJsonObject(Placement.FIELD_POSITION))
				: new Point();
		double rotation = jsonObject.containsKey(Placement.FIELD_ROTATION) ? jsonObject
				.getJsonNumber(Placement.FIELD_ROTATION).doubleValue() : 0.0;

		return new Placement(item, room, surface, referenceFrameId, position, rotation);
	}
}
