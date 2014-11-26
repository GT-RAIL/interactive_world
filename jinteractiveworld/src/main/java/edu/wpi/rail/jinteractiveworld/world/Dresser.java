package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Dresser object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Dresser extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Dresser";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 1.353;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.48;

	/**
	 * Create a new Dresser object.
	 */
	public Dresser() {
		super(Dresser.NAME, Dresser.WIDTH, Dresser.HEIGHT);
	}
}
