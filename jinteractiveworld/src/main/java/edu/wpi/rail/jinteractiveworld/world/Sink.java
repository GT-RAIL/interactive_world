package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Sink object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Sink extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Sink";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 0.82;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.46;

	/**
	 * Create a new Sink object.
	 */
	public Sink() {
		super(Sink.NAME, Sink.WIDTH, Sink.HEIGHT);
	}
}
