package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import edu.wpi.rail.jrosbridge.messages.Message;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonObject;
import java.io.StringReader;
import java.util.Arrays;

/**
 * The interactive_world_msgs/TaskModels message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 2, 2014
 */
public class TaskModels extends Message {

	/**
	 * The name of the models field for the message.
	 */
	public static final String FIELD_MODELS = "models";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/TaskModels";

	private final Model[] models;

	/**
	 * Create a new empty TaskModels message.
	 */
	public TaskModels() {
		this(new Model[0]);
	}

	/**
	 * Create a new TaskModels message based on the given information.
	 *
	 * @param models
	 *            The models of the TaskModels.
	 */
	public TaskModels(Model[] models) {
		// build the JSON object
		super(Json.createObjectBuilder().add(TaskModels.FIELD_MODELS,
				Json.createReader(new StringReader(Arrays.deepToString(models))).readArray())
				.build(), TaskModels.TYPE);
		this.models = models;
	}

	/**
	 * Get the models value of this TaskModels.
	 *
	 * @return The models value of this TaskModels.
	 */
	public Model[] getDate() {
		return this.models;
	}

	/**
	 * Create a clone of this TaskModels.
	 */
	@Override
	public TaskModels clone() {
		return new TaskModels(this.models);
	}

	/**
	 * Create a new TaskModels based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Point message based on the given JSON string.
	 */
	public static TaskModels fromJsonString(String jsonString) {
		// convert to a message
		return TaskModels.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new TaskModels based on the given Message. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Point message based on the given Message.
	 */
	public static TaskModels fromMessage(Message m) {
		// get it from the JSON object
		return TaskModels.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new TaskModels based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static TaskModels fromJsonObject(JsonObject jsonObject) {
		// check the fields
		JsonArray jsonModels = jsonObject.getJsonArray(TaskModels.FIELD_MODELS);
		if(jsonModels == null) {
			return new TaskModels(new Model[0]);
		} else {
			Model[] models = new Model[jsonModels.size()];
			for(int i = 0; i < models.length; ++i) {
				models[i] = Model.fromJsonObject(jsonModels.getJsonObject(i));
			}
			return new TaskModels(models);
		}
	}
}
