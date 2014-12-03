package edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs;

import edu.wpi.rail.jrosbridge.messages.Message;

import javax.json.Json;
import javax.json.JsonObject;

/**
 * The interactive_world_msgs/Model message.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version August 14, 2014
 */
public class Model extends Message {

	/**
	 * The name of the placement field for the message.
	 */
	public static final String FIELD_PLACEMENT = "placement";

	/**
	 * The name of the decision value field for the message.
	 */
	public static final String FIELD_DECISION_VALUE = "decision_value";

	/**
	 * The name of the standard deviation on the X field for the message.
	 */
	public static final String FIELD_SIGMA_X = "sigma_x";

	/**
	 * The name of the standard deviation on the Y field for the message.
	 */
	public static final String FIELD_SIGMA_Y = "sigma_y";

	/**
	 * The name of the standard deviation on the Z field for the message.
	 */
	public static final String FIELD_SIGMA_Z = "sigma_z";

	/**
	 * The name of the standard deviation on the theta field for the message.
	 */
	public static final String FIELD_SIGMA_THETA = "sigma_theta";

	/**
	 * The message type.
	 */
	public static final String TYPE = "interactive_world_msgs/Model";

	private final Placement placement;
	private final double decisionValue, sigmaX, sigmaY, sigmaZ, sigmaTheta;

	/**
	 * Create a new empty Model message.
	 */
	public Model() {
		this(new Placement(), 0.0, 0.0, 0.0, 0.0, 0.0);
	}

	/**
	 * Create a new Model message from an interactive world model.
	 *
	 * @param model
	 *            The model.
	 */
	public Model(edu.wpi.rail.jinteractiveworld.model.Model model) {
		this(model.getPlacementLocation(), model.getDecisionValue(), model.getSigmaX(), model.getSigmaY(), model.getSigmaZ(), model.getSigmaTheta());
	}

	/**
	 * Create a new Model message with the given information.
	 * 
	 * @param placement
	 *            The placement value of the message.
	 * @param decisionValue
	 *            The decision value of the message.
	 * @param sigmaX
	 *            The sigma X value of the message.
	 * @param sigmaY
	 *            The sigma Y value of the message.
	 * @param sigmaZ
	 *            The sigma Z value of the message.
	 * @param sigmaTheta
	 *            The sigma theta value of the message.
	 */
	public Model(Placement placement, double decisionValue, double sigmaX, double sigmaY, double sigmaZ, double sigmaTheta) {
		// build the JSON object
		super(Json.createObjectBuilder().add(Model.FIELD_PLACEMENT, placement.toJsonObject())
				.add(Model.FIELD_DECISION_VALUE, decisionValue)
				.add(Model.FIELD_SIGMA_X, sigmaX).add(Model.FIELD_SIGMA_Y, sigmaY)
				.add(Model.FIELD_SIGMA_Z, sigmaZ).add(Model.FIELD_SIGMA_THETA, sigmaTheta).build(), Model.TYPE);
		this.placement = placement;
		this.decisionValue = decisionValue;
		this.sigmaX = sigmaX;
		this.sigmaY = sigmaY;
		this.sigmaZ = sigmaZ;
		this.sigmaTheta = sigmaTheta;
	}

	/**
	 * Get the placement value of this Model.
	 * 
	 * @return The placement value of this Model.
	 */
	public Placement getPlacement() {
		return this.placement;
	}

	/**
	 * Get the decision value of this Model.
	 * 
	 * @return The decision value of this Model.
	 */
	public double getDecisionValue() {
		return this.decisionValue;
	}

	/**
	 * Get the sigma X value of this Model.
	 *
	 * @return The sigma X value of this Model.
	 */
	public double getSigmaX() {
		return this.sigmaX;
	}

	/**
	 * Get the sigma Y value of this Model.
	 *
	 * @return The sigma Y value of this Model.
	 */
	public double getSigmaY() {
		return this.sigmaY;
	}

	/**
	 * Get the sigma Z value of this Model.
	 *
	 * @return The sigma Z value of this Model.
	 */
	public double getSigmaZ() {
		return this.sigmaZ;
	}

	/**
	 * Get the sigma theta value of this Model.
	 *
	 * @return The sigma theta  value of this Model.
	 */
	public double getSigmaTheta() {
		return this.sigmaTheta;
	}

	/**
	 * Create a clone of this Model.
	 */
	@Override
	public Model clone() {
		return new Model(this.placement, this.decisionValue, this.sigmaX, this.sigmaY, this.sigmaZ, this.sigmaTheta);
	}

	/**
	 * Create a new Model based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Point message based on the given JSON string.
	 */
	public static Model fromJsonString(String jsonString) {
		// convert to a message
		return Model.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new Model based on the given Message. Any missing values will
	 * be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Point message based on the given Message.
	 */
	public static Model fromMessage(Message m) {
		// get it from the JSON object
		return Model.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new Model based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Point message based on the given JSON object.
	 */
	public static Model fromJsonObject(JsonObject jsonObject) {
		// check the fields
		Placement placement = jsonObject.containsKey(Model.FIELD_PLACEMENT) ? Placement
				.fromJsonObject(jsonObject.getJsonObject(Model.FIELD_PLACEMENT))
				: new Placement();
		double decisionValue = jsonObject.containsKey(Model.FIELD_DECISION_VALUE) ? jsonObject
				.getJsonNumber(Model.FIELD_DECISION_VALUE).doubleValue() : 0.0;
		double sigmaX = jsonObject.containsKey(Model.FIELD_SIGMA_X) ? jsonObject
				.getJsonNumber(Model.FIELD_SIGMA_X).doubleValue() : 0.0;
		double sigmaY = jsonObject.containsKey(Model.FIELD_SIGMA_Y) ? jsonObject
				.getJsonNumber(Model.FIELD_SIGMA_Y).doubleValue() : 0.0;
		double sigmaZ = jsonObject.containsKey(Model.FIELD_SIGMA_Z) ? jsonObject
				.getJsonNumber(Model.FIELD_SIGMA_Z).doubleValue() : 0.0;
		double sigmaTheta = jsonObject.containsKey(Model.FIELD_SIGMA_THETA) ? jsonObject
				.getJsonNumber(Model.FIELD_SIGMA_THETA).doubleValue() : 0.0;
		return new Model(placement, decisionValue, sigmaX, sigmaY, sigmaZ, sigmaTheta);
	}
}
