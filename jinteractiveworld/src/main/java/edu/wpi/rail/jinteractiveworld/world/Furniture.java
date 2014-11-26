package edu.wpi.rail.jinteractiveworld.world;

/**
 * A Furniture is an abstract class that will be the parent class of any real
 * world furniture object in the interactive world such as a table.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 17, 2014
 */
public abstract class Furniture extends Object {

	/**
	 * Create a new Furniture with the given name.
	 * 
	 * @param name
	 *            The name of the Furniture.
	 * @param width
	 *            The width the Furniture.
	 * @param height
	 *            The height of the Furniture.
	 */
	public Furniture(String name, double width, double height) {
		super(name, width, height);
	}
}