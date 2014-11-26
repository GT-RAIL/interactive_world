package edu.wpi.rail.jinteractiveworld.data;

import edu.wpi.rail.jinteractiveworld.model.transform.RotationMatrix;
import edu.wpi.rail.jinteractiveworld.model.transform.Transform;
import edu.wpi.rail.jinteractiveworld.model.transform.Vector3;
import edu.wpi.rail.jinteractiveworld.world.Bed;
import edu.wpi.rail.jinteractiveworld.world.Burner;
import edu.wpi.rail.jinteractiveworld.world.Cabinet;
import edu.wpi.rail.jinteractiveworld.world.Chair;
import edu.wpi.rail.jinteractiveworld.world.CoffeeTable;
import edu.wpi.rail.jinteractiveworld.world.Couch;
import edu.wpi.rail.jinteractiveworld.world.Counter;
import edu.wpi.rail.jinteractiveworld.world.Cup;
import edu.wpi.rail.jinteractiveworld.world.DiningTable;
import edu.wpi.rail.jinteractiveworld.world.Dresser;
import edu.wpi.rail.jinteractiveworld.world.Fork;
import edu.wpi.rail.jinteractiveworld.world.Item;
import edu.wpi.rail.jinteractiveworld.world.Magazines;
import edu.wpi.rail.jinteractiveworld.world.Nightstand;
import edu.wpi.rail.jinteractiveworld.world.Object;
import edu.wpi.rail.jinteractiveworld.world.Oven;
import edu.wpi.rail.jinteractiveworld.world.Pillow;
import edu.wpi.rail.jinteractiveworld.world.Plate;
import edu.wpi.rail.jinteractiveworld.world.Refrigerator;
import edu.wpi.rail.jinteractiveworld.world.Sink;
import edu.wpi.rail.jinteractiveworld.world.SinkUnit;
import edu.wpi.rail.jinteractiveworld.world.Speaker;
import edu.wpi.rail.jinteractiveworld.world.Spoon;
import edu.wpi.rail.jinteractiveworld.world.TV;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import weka.core.Attribute;
import weka.core.FastVector;
import weka.core.Instance;
import weka.core.Instances;

/**
 * A DataSet contains a collection of transformation matrices. Rotations occur
 * along the Z axis.
 * 
 * @author Russell Toris -- rctoris@wpi.edu
 * @version February 6, 2014
 */
public class DataSet {

	/**
	 * The acceptable file extension for a data set file.
	 */
	public static final String FILE_EXTENSION = ".csv";

	/**
	 * The dimensionality of the data set.
	 */
	public static final int N = 4;

	/**
	 * The x attribute for the data set.
	 */
	public static final Attribute X_ATTRIBUTE = new Attribute("x");

	/**
	 * The y attribute for the data set.
	 */
	public static final Attribute Y_ATTRIBUTE = new Attribute("y");

	/**
	 * The z attribute for the data set.
	 */
	public static final Attribute Z_ATTRIBUTE = new Attribute("z");

	/**
	 * The theta attribute for the data set.
	 */
	public static final Attribute THETA_ATTRIBUTE = new Attribute("theta");

	private ArrayList<Transform> data;
	private String name;

	/**
	 * Create a new, empty data set and empty name.
	 */
	public DataSet() {
		this("");
	}

	/**
	 * Create a new, empty data set with the given name.
	 * 
	 * @param name
	 *            The name of the data set.
	 */
	public DataSet(String name) {
		this.data = new ArrayList<Transform>();
		this.name = name;
	}

	/**
	 * Create a new data set from the given CSV file. Errors are reported to the
	 * terminal as warnings. The name will be the file name.
	 * 
	 * @param csv
	 *            The CSV file to load.
	 */
	public DataSet(File csv) {
		// create the array list
		this(csv.getAbsolutePath());

		try {
			// open a writer
			BufferedReader br = new BufferedReader(new FileReader(
					csv.getAbsoluteFile()));

			// add the new points
			String line;
			while ((line = br.readLine()) != null) {
				// parse the values
				String[] values = line.split(",");
				try {
					double x = Double.parseDouble(values[0]);
					double y = Double.parseDouble(values[1]);
					double z = Double.parseDouble(values[2]);
					double theta = Double.parseDouble(values[3]);
					// add it to the list
					this.add(x, y, z, theta);
				} catch (NumberFormatException e) {
					// bad file format -- print a warning
					System.err.println("[WARN]: Bad formating in file \"" + csv
							+ "\" with line \"" + line + "\".");
				} catch (IndexOutOfBoundsException e2) {
					// bad file format -- print a warning
					System.err.println("[WARN]: Bad formating in file \"" + csv
							+ "\" with line \"" + line + "\".");
				}
			}

			br.close();
		} catch (IOException e3) {
			// bad file -- print a warning
			System.err.println("[WARN]: Could not load file \"" + csv + "\".");
		}
	}

	/**
	 * Get the name of the data set.
	 * 
	 * @return The name of the data set.
	 */
	public String getName() {
		return this.name;
	}

	/**
	 * Set the name of the data set.
	 * 
	 * @param name
	 *            The name of the data set.
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * Get the size of this data set.
	 * 
	 * @return The number of points in the data set.
	 */
	public int size() {
		return this.data.size();
	}

	/**
	 * Add a data point based on the given x, y, z, and theta values.
	 * 
	 * @param x
	 *            The x value of the data point.
	 * @param y
	 *            The y value of the data point.
	 * @param z
	 *            The z value of the data point.
	 * @param theta
	 *            The theta value of the data point.
	 */
	public void add(double x, double y, double z, double theta) {
		RotationMatrix r = new RotationMatrix(theta,
				RotationMatrix.RotationType.Z_ROTATION);
		Vector3 v = new Vector3(x, y, z);

		this.add(new Transform(r, v));
	}

	/**
	 * Add the given transform to the data set.
	 * 
	 * @param tf
	 *            The transform to add.
	 */
	public void add(Transform tf) {
		this.data.add(tf);
	}

	/**
	 * Get the data point (as a matrix) at the given index.
	 * 
	 * @param index
	 *            The index to get.
	 * @return The data at the given index.
	 */
	public Transform get(int index) {
		return this.data.get(index);
	}

	/**
	 * Get the data set as a set of Weka instances.
	 * 
	 * @return The data set as a set of Weka instances.
	 */
	public Instances toInstances() {
		// attributes for the set
		FastVector attributes = new FastVector(DataSet.N);
		attributes.addElement(DataSet.X_ATTRIBUTE);
		attributes.addElement(DataSet.Y_ATTRIBUTE);
		attributes.addElement(DataSet.Z_ATTRIBUTE);
		attributes.addElement(DataSet.THETA_ATTRIBUTE);

		Instances instances = new Instances("data", attributes, this.size());
		// add each instance
		for (int i = 0; i < this.size(); i++) {
			// get the TF
			Transform tf = this.get(i);
			Vector3 v = tf.getVector3();
			double theta = tf.getRotationMatrix().getRotationAboutAxis(
					RotationMatrix.RotationType.Z_ROTATION);

			Instance inst = new Instance(DataSet.N);
			inst.setDataset(instances);
			// x, y, z, theta
			inst.setValue(DataSet.X_ATTRIBUTE, v.getX());
			inst.setValue(DataSet.Y_ATTRIBUTE, v.getY());
			inst.setValue(DataSet.Z_ATTRIBUTE, v.getZ());
			inst.setValue(DataSet.THETA_ATTRIBUTE, theta);

			instances.add(inst);
		}

		return instances;
	}

	/**
	 * Get the minimum x value.
	 * 
	 * @return The minimum x value.
	 */
	public double getMinX() {
		double minX = Double.POSITIVE_INFINITY;
		for (Transform tf : this.data) {
			Vector3 v = tf.getVector3();
			if (v.getX() < minX) {
				minX = v.getX();
			}
		}
		return minX;
	}

	/**
	 * Get the maximum x value.
	 * 
	 * @return The maximum x value.
	 */
	public double getMaxX() {
		double maxX = Double.NEGATIVE_INFINITY;
		for (Transform tf : this.data) {
			Vector3 v = tf.getVector3();
			if (v.getX() > maxX) {
				maxX = v.getX();
			}
		}
		return maxX;
	}

	/**
	 * Get the minimum y value.
	 * 
	 * @return The minimum y value.
	 */
	public double getMinY() {
		double minY = Double.POSITIVE_INFINITY;
		for (Transform tf : this.data) {
			Vector3 v = tf.getVector3();
			if (v.getY() < minY) {
				minY = v.getY();
			}
		}
		return minY;
	}

	/**
	 * Get the maximum y value.
	 * 
	 * @return The maximum y value.
	 */
	public double getMaxY() {
		double maxY = Double.NEGATIVE_INFINITY;
		for (Transform tf : this.data) {
			Vector3 v = tf.getVector3();
			if (v.getY() > maxY) {
				maxY = v.getY();
			}
		}
		return maxY;
	}

	/**
	 * Get the minimum z value.
	 * 
	 * @return The minimum z value.
	 */
	public double getMinZ() {
		double minZ = Double.POSITIVE_INFINITY;
		for (Transform tf : this.data) {
			Vector3 v = tf.getVector3();
			if (v.getZ() < minZ) {
				minZ = v.getZ();
			}
		}
		return minZ;
	}

	/**
	 * Get the maximum z value.
	 * 
	 * @return The maximum z value.
	 */
	public double getMaxZ() {
		double maxZ = Double.NEGATIVE_INFINITY;
		for (Transform tf : this.data) {
			Vector3 v = tf.getVector3();
			if (v.getZ() > maxZ) {
				maxZ = v.getZ();
			}
		}
		return maxZ;
	}

	/**
	 * Load the given folder of data sets (*.csv).
	 * 
	 * @param dir
	 *            The directory to load.
	 * @return All data sets in the given folder.
	 */
	public static ArrayList<DataSet> loadDataFolder(String folder) {
		File dir = new File(folder);
		ArrayList<DataSet> data = new ArrayList<DataSet>();

		// get the files
		File[] files = dir.listFiles();
		for (File f : files) {
			// check the file extension
			if (f.getName().endsWith(DataSet.FILE_EXTENSION)) {
				data.add(new DataSet(f));
			}
		}

		return data;
	}

	/**
	 * Attempt to extract the target item from the data set name. This will only
	 * work if the name corresponds to a file name created by the parser.
	 * 
	 * @param ds
	 *            The data set.
	 * @return The target item extracted from the name (if there is one).
	 */
	public static Item extractTargetFromName(DataSet ds) {
		String name = ds.getName();

		try {
			// pull out the actual name of the file
			String fName = name
					.substring(name.lastIndexOf("/") + 1,
							name.lastIndexOf(".csv")).toLowerCase()
					.replaceAll(" ", "");

			Cup cup = new Cup();
			Fork fork = new Fork();
			Magazines magazines = new Magazines();
			Plate plate = new Plate();
			Spoon spoon = new Spoon();

			// work backwards
			if (fName.endsWith(cup.getName().toLowerCase().replaceAll(" ", ""))) {
				return cup;
			} else if (fName.endsWith(fork.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return fork;
			} else if (fName.endsWith(magazines.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return magazines;
			} else if (fName.endsWith(plate.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return plate;
			} else if (fName.endsWith(spoon.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return spoon;
			} else {
				return null;
			}
		} catch (StringIndexOutOfBoundsException e) {
			// invalid name
			return null;
		}
	}

	/**
	 * Attempt to extract the reference object from the data set name. This will
	 * only work if the name corresponds to a file name created by the parser.
	 * 
	 * @param ds
	 *            The data set.
	 * @return The reference object extracted from the name (if there is one).
	 */
	public static Object extractReferenceFromName(DataSet ds) {
		// check for the target first
		Item target = DataSet.extractTargetFromName(ds);
		if (target != null) {
			// pull out the actual name of the file without the target
			String name = ds.getName();
			String fName = name
					.substring(name.lastIndexOf("/") + 1,
							name.lastIndexOf(".csv"))
					.toLowerCase()
					.replaceAll(" ", "")
					.replace(
							target.getName().toLowerCase().replaceAll(" ", ""),
							"");

			Bed bed = new Bed();
			Burner burner = new Burner();
			Cabinet cabinet = new Cabinet();
			Chair chair = new Chair();
			CoffeeTable coffeeTable = new CoffeeTable();
			Couch couch = new Couch();
			Counter counter = new Counter();
			Cup cup = new Cup();
			DiningTable diningTable = new DiningTable();
			Dresser dresser = new Dresser();
			Fork fork = new Fork();
			Magazines magazines = new Magazines();
			Nightstand nightstand = new Nightstand();
			Oven oven = new Oven();
			Pillow pillow = new Pillow();
			Plate plate = new Plate();
			Refrigerator refrigerator = new Refrigerator();
			Sink sink = new Sink();
			SinkUnit sinkUnit = new SinkUnit();
			Speaker speaker = new Speaker();
			Spoon spoon = new Spoon();
			TV tv = new TV();

			// work backwards
			if (fName.endsWith(bed.getName().toLowerCase().replaceAll(" ", ""))) {
				return bed;
			} else if (fName.endsWith(burner.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return burner;
			} else if (fName.endsWith(cabinet.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return cabinet;
			} else if (fName.endsWith(chair.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return chair;
			} else if (fName.endsWith(coffeeTable.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return coffeeTable;
			} else if (fName.endsWith(couch.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return couch;
			} else if (fName.endsWith(counter.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return counter;
			} else if (fName.endsWith(cup.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return cup;
			} else if (fName.endsWith(diningTable.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return diningTable;
			} else if (fName.endsWith(dresser.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return dresser;
			} else if (fName.endsWith(fork.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return fork;
			} else if (fName.endsWith(magazines.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return magazines;
			} else if (fName.endsWith(nightstand.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return nightstand;
			} else if (fName.endsWith(oven.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return oven;
			} else if (fName.endsWith(pillow.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return pillow;
			} else if (fName.endsWith(plate.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return plate;
			} else if (fName.endsWith(refrigerator.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return refrigerator;
			} else if (fName.endsWith(sink.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return sink;
			} else if (fName.endsWith(sinkUnit.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return sinkUnit;
			} else if (fName.endsWith(speaker.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return speaker;
			} else if (fName.endsWith(spoon.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return spoon;
			} else if (fName.endsWith(tv.getName().toLowerCase()
					.replaceAll(" ", ""))) {
				return tv;
			} else {
				return null;
			}
		} else {
			return null;
		}
	}
}
