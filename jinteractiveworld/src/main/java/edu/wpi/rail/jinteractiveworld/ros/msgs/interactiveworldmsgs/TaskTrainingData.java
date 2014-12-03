package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import edu.wpi.rail.jrosbridge.messages.Message;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonObject;
import java.io.StringReader;
import java.util.Arrays;

/**
 * The interactive_world_msgs/TaskTrainingData message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 2, 2014
 */
public class TaskTrainingData extends Message {

	/**
	 * The name of the data field for the message.
	 */
	public static final String FIELD_DATA = "data";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/TaskTrainingData";

	private final PlacementSet[] data;

	/**
	 * Create a new empty TaskTrainingData message.
	 */
	public TaskTrainingData() {
		this(new PlacementSet[0]);
	}

	/**
	 * Create a new TaskTrainingData message based on the given information.
	 *
	 * @param data
	 *            The data of the TaskTrainingData.
	 */
	public TaskTrainingData(PlacementSet[] data) {
		// build the JSON object
		super(Json.createObjectBuilder().add(TaskTrainingData.FIELD_DATA,
				Json.createReader(new StringReader(Arrays.deepToString(data))).readArray())
				.build(), TaskTrainingData.TYPE);
		this.data = data;
	}

	/**
	 * Get the data value of this TaskTrainingData.
	 *
	 * @return The data value of this TaskTrainingData.
	 */
	public PlacementSet[] getDate() {
		return this.data;
	}

	/**
	 * Create a clone of this TaskTrainingData.
	 */
	@Override
	public TaskTrainingData clone() {
		return new TaskTrainingData(this.data);
	}

	/**
	 * Create a new TaskTrainingData based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Point message based on the given JSON string.
	 */
	public static TaskTrainingData fromJsonString(String jsonString) {
		// convert to a message
		return TaskTrainingData.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new TaskTrainingData based on the given Message. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Point message based on the given Message.
	 */
	public static TaskTrainingData fromMessage(Message m) {
		// get it from the JSON object
		return TaskTrainingData.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new TaskTrainingData based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static TaskTrainingData fromJsonObject(JsonObject jsonObject) {
		// check the fields
		JsonArray jsonData = jsonObject.getJsonArray(TaskTrainingData.FIELD_DATA);
		if(jsonData == null) {
			return new TaskTrainingData(new PlacementSet[0]);
		} else {
			PlacementSet[] data = new PlacementSet[jsonData.size()];
			for(int i = 0; i < data.length; ++i) {
				data[i] = PlacementSet.fromJsonObject(jsonData.getJsonObject(i));
			}
			return new TaskTrainingData(data);
		}
	}
}
