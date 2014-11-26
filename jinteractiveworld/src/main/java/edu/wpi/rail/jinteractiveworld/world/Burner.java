package edu.wpi.rail.jinteractiveworld.world;

/**
 * The Burner object in the world.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version March 3, 2014
 */
public class Burner extends Furniture {

	/**
	 * The name of the furniture.
	 */
	private static final String NAME = "Burner";

	/**
	 * The width of the furniture.
	 */
	private static final double WIDTH = 0.25;

	/**
	 * The height of the furniture.
	 */
	private static final double HEIGHT = 0.25;

	/**
	 * Create a new Burner object.
	 */
	public Burner() {
		super(Burner.NAME, Burner.WIDTH, Burner.HEIGHT);
	}
}
