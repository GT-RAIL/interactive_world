package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonObject;
import java.io.StringReader;
import java.util.Arrays;

/**
 * The interactive_world_msgs/PlacementSet message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 2, 2014
 */
public class PlacementSet extends Message {

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
	 * The name of the placements field for the message.
	 */
	public static final String FIELD_PLACEMENTS = "placements";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/PlacementSet";

	private final Item item;
	private final Room room;
	private final Surface surface;
	private final String referenceFrameId;
	private final Placement[] placements;

	/**
	 * Create a new empty PlacementSet message.
	 */
	public PlacementSet() {
		this(new Item(), new Room(), new Surface(), "", new Placement[0]);
	}

	/**
	 * Create a new PlacementSet message based on the given information.
	 *
	 * @param item
	 *            The item of the PlacementSet.
	 * @param room
	 *            The room of the PlacementSet.
	 * @param surface
	 *            The surface of the PlacementSet.
	 * @param referenceFrameId
	 *            The reference frame of the PlacementSet.
	 * @param placements
	 *            The placements of the PlacementSet.
	 */
	public PlacementSet(Item item, Room room, Surface surface, String referenceFrameId, Placement[] placements) {
		// build the JSON object
		super(Json.createObjectBuilder().add(PlacementSet.FIELD_ITEM, item.toJsonObject())
				.add(PlacementSet.FIELD_ROOM, room.toJsonObject())
				.add(PlacementSet.FIELD_SURFACE, surface.toJsonObject())
				.add(PlacementSet.FIELD_REFERENCE_FRAME_ID, referenceFrameId)
				.add(PlacementSet.FIELD_PLACEMENTS, Json.createReader(new StringReader(Arrays.deepToString(placements))).readArray())
				.build(), PlacementSet.TYPE);
		this.item = item;
		this.room = room;
		this.surface = surface;
		this.referenceFrameId = referenceFrameId;
		this.placements = placements;
	}

	/**
	 * Get the item value of this PlacementSet.
	 *
	 * @return The item value of this PlacementSet.
	 */
	public Item getItem() {
		return this.item;
	}

	/**
	 * Get the room value of this PlacementSet.
	 *
	 * @return The room value of this PlacementSet.
	 */
	public Room getRoom() {
		return this.room;
	}

	/**
	 * Get the surface value of this PlacementSet.
	 *
	 * @return The surface value of this PlacementSet.
	 */
	public Surface getSurface() {
		return this.surface;
	}

	/**
	 * Get the reference frame value of this PlacementSet.
	 *
	 * @return The reference frame value of this PlacementSet.
	 */
	public String getReferenceFrameId() {
		return this.referenceFrameId;
	}

	/**
	 * Get the placements value of this PlacementSet.
	 *
	 * @return The placements value of this PlacementSet.
	 */
	public Placement[] getPlacements() {
		return this.placements;
	}

	/**
	 * Create a clone of this PlacementSet.
	 */
	@Override
	public PlacementSet clone() {
		return new PlacementSet(this.item, this.room, this.surface, this.referenceFrameId, this.placements);
	}

	/**
	 * Create a new PlacementSet based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Point message based on the given JSON string.
	 */
	public static PlacementSet fromJsonString(String jsonString) {
		// convert to a message
		return PlacementSet.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new PlacementSet based on the given Message. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Point message based on the given Message.
	 */
	public static PlacementSet fromMessage(Message m) {
		// get it from the JSON object
		return PlacementSet.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new PlacementSet based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static PlacementSet fromJsonObject(JsonObject jsonObject) {
		// check the fields
		Item item = jsonObject.containsKey(PlacementSet.FIELD_ITEM) ? Item
				.fromJsonObject(jsonObject.getJsonObject(PlacementSet.FIELD_ITEM))
				: new Item();
		Room room = jsonObject.containsKey(PlacementSet.FIELD_ROOM) ? Room
				.fromJsonObject(jsonObject.getJsonObject(PlacementSet.FIELD_ROOM))
				: new Room();
		Surface surface = jsonObject.containsKey(PlacementSet.FIELD_SURFACE) ? Surface
				.fromJsonObject(jsonObject.getJsonObject(PlacementSet.FIELD_SURFACE))
				: new Surface();
		String referenceFrameId = jsonObject.containsKey(PlacementSet.FIELD_REFERENCE_FRAME_ID) ? jsonObject
				.getString(PlacementSet.FIELD_REFERENCE_FRAME_ID) : "";
		JsonArray jsonPlacements = jsonObject.getJsonArray(PlacementSet.FIELD_PLACEMENTS);
		if(jsonPlacements == null) {
			return new PlacementSet(item, room, surface, referenceFrameId, new Placement[0]);
		} else {
			Placement[] placements = new Placement[jsonPlacements.size()];
			for(int i = 0; i < placements.length; ++i) {
				placements[i] = Placement.fromJsonObject(jsonPlacements.getJsonObject(i));
			}
			return new PlacementSet(item, room, surface, referenceFrameId, placements);
		}
	}
}
