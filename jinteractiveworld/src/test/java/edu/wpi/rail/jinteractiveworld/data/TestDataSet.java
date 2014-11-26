package edu.wpi.rail.jinteractiveworld.data;

import static org.junit.Assert.*;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import org.junit.Test;

import weka.core.Instance;
import edu.wpi.rail.jinteractiveworld.data.DataSet;
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
import edu.wpi.rail.jinteractiveworld.world.Magazines;
import edu.wpi.rail.jinteractiveworld.world.Nightstand;
import edu.wpi.rail.jinteractiveworld.world.Oven;
import edu.wpi.rail.jinteractiveworld.world.Pillow;
import edu.wpi.rail.jinteractiveworld.world.Plate;
import edu.wpi.rail.jinteractiveworld.world.Refrigerator;
import edu.wpi.rail.jinteractiveworld.world.Sink;
import edu.wpi.rail.jinteractiveworld.world.SinkUnit;
import edu.wpi.rail.jinteractiveworld.world.Speaker;
import edu.wpi.rail.jinteractiveworld.world.Spoon;
import edu.wpi.rail.jinteractiveworld.world.TV;

public class TestDataSet {

	@Test
	public void testConstructor() {
		DataSet ds = new DataSet();
		assertEquals(0, ds.size());
		assertEquals("", ds.getName());
	}

	@Test
	public void testNameConstructor() {
		String name = "test";
		DataSet ds = new DataSet(name);
		assertEquals(0, ds.size());
		assertEquals(name, ds.getName());
	}

	@Test
	public void testCsvConstructor() throws IOException {
		String fName = "test.txt";
		File f = new File("test.txt");
		f.createNewFile();

		BufferedWriter bw = new BufferedWriter(new FileWriter(
				f.getAbsoluteFile()));
		bw.write("10.1,20.1,50.1,3.14\r\n");
		bw.write("20.1,30.1,60.1,2.16\r\n");
		bw.close();

		DataSet ds = new DataSet(new File(fName));
		assertEquals(2, ds.size());
		assertEquals(new Transform(new RotationMatrix(3.14,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(10.1,
				20.1, 50.1)), ds.get(0));
		assertEquals(new Transform(new RotationMatrix(2.16,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(20.1,
				30.1, 60.1)), ds.get(1));
		assertEquals(f.getAbsolutePath(), ds.getName());

		f.delete();
	}

	@Test
	public void testCsvConstructorNoFile() {
		DataSet ds = new DataSet(new File("invalid"));
		assertEquals(0, ds.size());
	}

	@Test
	public void testCsvConstructorNoDoubles() throws IOException {
		String fName = "test.txt";
		File f = new File("test.txt");
		f.createNewFile();

		BufferedWriter bw = new BufferedWriter(new FileWriter(
				f.getAbsoluteFile()));
		bw.write("10.1,20.1,50.1,3.14\r\n");
		bw.write("BAD,FILE\r\n");
		bw.write("20.1,30.1,60.1,2.16\r\n");
		bw.close();

		DataSet ds = new DataSet(new File(fName));
		assertEquals(2, ds.size());
		assertEquals(new Transform(new RotationMatrix(3.14,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(10.1,
				20.1, 50.1)), ds.get(0));
		assertEquals(new Transform(new RotationMatrix(2.16,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(20.1,
				30.1, 60.1)), ds.get(1));

		f.delete();
	}

	@Test
	public void testCsvConstructorNoTheta() throws IOException {
		String fName = "test.txt";
		File f = new File("test.txt");
		f.createNewFile();

		BufferedWriter bw = new BufferedWriter(new FileWriter(
				f.getAbsoluteFile()));
		bw.write("10.1,20.1,50.1,3.14\r\n");
		bw.write("-20.1,-30.1,10.4\r\n");
		bw.write("20.1,30.1,60.1,2.16\r\n");
		bw.close();

		DataSet ds = new DataSet(new File(fName));
		assertEquals(2, ds.size());
		assertEquals(new Transform(new RotationMatrix(3.14,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(10.1,
				20.1, 50.1)), ds.get(0));
		assertEquals(new Transform(new RotationMatrix(2.16,
				RotationMatrix.RotationType.Z_ROTATION), new Vector3(20.1,
				30.1, 60.1)), ds.get(1));

		f.delete();
	}

	@Test
	public void testSetName() {
		String name1 = "test1";
		DataSet ds = new DataSet(name1);
		String name2 = "test2";
		ds.setName(name2);
		assertEquals(name2, ds.getName());
	}

	@Test
	public void testAdd() {
		DataSet ds = new DataSet();
		assertEquals(0, ds.size());

		double x1 = 0.5;
		double y1 = -0.5;
		double z1 = 5.5;
		double theta1 = Math.PI / 2.0;
		ds.add(x1, y1, z1, theta1);
		assertEquals(1, ds.size());
		assertEquals(new Transform(new RotationMatrix(theta1,
				RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(x1, y1, z1)), ds.get(0));
		double x2 = 0.25;
		double y2 = -0.25;
		double z2 = -5.5;
		double theta2 = Math.PI / 4.0;
		ds.add(x2, y2, z2, theta2);
		assertEquals(2, ds.size());
		assertEquals(new Transform(new RotationMatrix(theta2,
				RotationMatrix.RotationType.Z_ROTATION),
				new Vector3(x2, y2, z2)), ds.get(1));
	}

	@Test
	public void testGetInstances() {
		DataSet ds = new DataSet();
		assertEquals(0, ds.size());

		double x1 = 0.5;
		double y1 = -0.5;
		double z1 = 5.5;
		double theta1 = Math.PI / 2.0;
		ds.add(x1, y1, z1, theta1);
		assertEquals(1, ds.size());
		assertEquals(DataSet.N, ds.toInstances().numAttributes());
		assertEquals(1, ds.toInstances().numInstances());
		Instance inst1 = ds.toInstances().instance(0);
		assertEquals(DataSet.N, inst1.numAttributes());
		assertEquals(x1, inst1.value(DataSet.X_ATTRIBUTE));
		assertEquals(y1, inst1.value(DataSet.Y_ATTRIBUTE));
		assertEquals(z1, inst1.value(DataSet.Z_ATTRIBUTE));
		assertEquals(theta1, inst1.value(DataSet.THETA_ATTRIBUTE));

		double x2 = 0.25;
		double y2 = -0.25;
		double z2 = -5.5;
		double theta2 = Math.PI / 4.0;
		ds.add(x2, y2, z2, theta2);
		assertEquals(2, ds.size());
		assertEquals(DataSet.N, ds.toInstances().numAttributes());
		assertEquals(2, ds.toInstances().numInstances());
		Instance inst2 = ds.toInstances().instance(1);
		assertEquals(DataSet.N, inst2.numAttributes());
		assertEquals(x2, inst2.value(DataSet.X_ATTRIBUTE));
		assertEquals(y2, inst2.value(DataSet.Y_ATTRIBUTE));
		assertEquals(z2, inst2.value(DataSet.Z_ATTRIBUTE));
		assertEquals(theta2, inst2.value(DataSet.THETA_ATTRIBUTE));
	}

	@Test
	public void testGetMinX() {
		DataSet ds = new DataSet();
		assertEquals(Double.POSITIVE_INFINITY, ds.getMinX(), 0);

		double x1 = 0.5;
		ds.add(x1, 0, 0, 0);
		assertEquals(x1, ds.getMinX(), 0);
		double x2 = 1.5;
		ds.add(x2, 0, 0, 0);
		assertEquals(x1, ds.getMinX(), 0);
		double x3 = -1.5;
		ds.add(x3, 0, 0, 0);
		assertEquals(x3, ds.getMinX(), 0);
	}

	@Test
	public void testGetMaxX() {
		DataSet ds = new DataSet();
		assertEquals(Double.NEGATIVE_INFINITY, ds.getMaxX(), 0);

		double x1 = 0.5;
		ds.add(x1, 0, 0, 0);
		assertEquals(x1, ds.getMaxX(), 0);
		double x2 = 1.5;
		ds.add(x2, 0, 0, 0);
		assertEquals(x2, ds.getMaxX(), 0);
		double x3 = -1.5;
		ds.add(x3, 0, 0, 0);
		assertEquals(x2, ds.getMaxX(), 0);
	}

	@Test
	public void testGetMinY() {
		DataSet ds = new DataSet();
		assertEquals(Double.POSITIVE_INFINITY, ds.getMinY(), 0);

		double y1 = 0.5;
		ds.add(0, y1, 0, 0);
		assertEquals(y1, ds.getMinY(), 0);
		double y2 = 1.5;
		ds.add(0, y2, 0, 0);
		assertEquals(y1, ds.getMinY(), 0);
		double y3 = -1.5;
		ds.add(0, y3, 0, 0);
		assertEquals(y3, ds.getMinY(), 0);
	}

	@Test
	public void testGetMaxY() {
		DataSet ds = new DataSet();
		assertEquals(Double.NEGATIVE_INFINITY, ds.getMaxY(), 0);

		double y1 = 0.5;
		ds.add(0, y1, 0, 0);
		assertEquals(y1, ds.getMaxY(), 0);
		double y2 = 1.5;
		ds.add(0, y2, 0, 0);
		assertEquals(y2, ds.getMaxY(), 0);
		double y3 = -1.5;
		ds.add(0, y3, 0, 0);
		assertEquals(y2, ds.getMaxY(), 0);
	}

	@Test
	public void testGetMinZ() {
		DataSet ds = new DataSet();
		assertEquals(Double.POSITIVE_INFINITY, ds.getMinZ(), 0);

		double z1 = 0.5;
		ds.add(0, 0, z1, 0);
		assertEquals(z1, ds.getMinZ(), 0);
		double z2 = 1.5;
		ds.add(0, 0, z2, 0);
		assertEquals(z1, ds.getMinZ(), 0);
		double z3 = -1.5;
		ds.add(0, 0, z3, 0);
		assertEquals(z3, ds.getMinZ(), 0);
	}

	@Test
	public void testGetMaxZ() {
		DataSet ds = new DataSet();
		assertEquals(Double.NEGATIVE_INFINITY, ds.getMaxZ(), 0);

		double z1 = 0.5;
		ds.add(0, 0, z1, 0);
		assertEquals(z1, ds.getMaxZ(), 0);
		double z2 = 1.5;
		ds.add(0, 0, z2, 0);
		assertEquals(z2, ds.getMaxZ(), 0);
		double z3 = -1.5;
		ds.add(0, 0, z3, 0);
		assertEquals(z2, ds.getMaxZ(), 0);
	}

	@Test
	public void testLoadDataFolderNoValid() {
		ArrayList<DataSet> data = DataSet.loadDataFolder("../");
		assertEquals(0, data.size());
	}

//	@Test
//	public void testLoadDataFolder() {
//		ArrayList<DataSet> data = DataSet.loadDataFolder("data/table_setting/");
//		assertEquals(20, data.size());
//		for (DataSet ds : data) {
//			assertEquals(1, ds.size() / ds.size());
//		}
//	}

	@Test
	public void testExtractTargetFromNameInvalid() {
		assertNull(DataSet.extractTargetFromName(new DataSet("invalid")));
	}

	@Test
	public void testExtractTargetFromNameCup() {
		assertEquals(new Cup(), DataSet.extractTargetFromName(new DataSet(
				"/test/file/DiningTablewithChairsForkCup.csv")));
	}

	@Test
	public void testExtractTargetFromNameFork() {
		assertEquals(new Fork(), DataSet.extractTargetFromName(new DataSet(
				"/test/file/DiningTablewithChairsCupFork.csv")));
	}

	@Test
	public void testExtractTargetFromNamePlate() {
		assertEquals(new Plate(), DataSet.extractTargetFromName(new DataSet(
				"/test/file/DiningTablewithChairsCupPlate.csv")));
	}

	@Test
	public void testExtractTargetFromNameMagazines() {
		assertEquals(new Magazines(),
				DataSet.extractTargetFromName(new DataSet(
						"/test/file/DiningTablewithChairsCupMagazines.csv")));
	}

	@Test
	public void testExtractTargetFromNameSpoon() {
		assertEquals(new Spoon(), DataSet.extractTargetFromName(new DataSet(
				"/test/file/DiningTablewithChairsCupSpoon.csv")));
	}

	@Test
	public void testExtractTargetFromNameInvalidItem() {
		assertNull(DataSet.extractTargetFromName(new DataSet(
				"/test/file/DiningTablewithChairsCupInvalid.csv")));
	}

	@Test
	public void testExtractReferenceFromNameInvalid() {
		assertNull(DataSet.extractReferenceFromName(new DataSet("invalid")));
	}

	@Test
	public void testExtractReferenceFromNameBed() {
		assertEquals(new Bed(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/BedCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameBurner() {
		assertEquals(new Burner(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/OvenburnerCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameCabinet() {
		assertEquals(new Cabinet(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/CabinetCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameChair() {
		assertEquals(new Chair(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/DiningTablewithChairschairCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameCoffeeTable() {
		assertEquals(new CoffeeTable(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/CoffeeTableCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameCouch() {
		assertEquals(new Couch(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/CouchCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameCounter() {
		assertEquals(new Counter(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/CounterCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameCup() {
		assertEquals(new Cup(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/DiningTablewithChairsCupFork.csv")));
	}

	@Test
	public void testExtractReferenceFromNameDiningTable() {
		assertEquals(new DiningTable(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/DiningTablewithChairsFork.csv")));
	}

	@Test
	public void testExtractReferenceFromNameDresser() {
		assertEquals(new Dresser(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/DresserCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameFork() {
		assertEquals(new Fork(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/DiningTablewithChairsForkCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameMagazines() {
		assertEquals(new Magazines(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/DiningTablewithChairsMagazinesCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameNightstand() {
		assertEquals(new Nightstand(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/NightstandCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameOven() {
		assertEquals(new Oven(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/OvenCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNamePillow() {
		assertEquals(new Pillow(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/BedpillowCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNamePlate() {
		assertEquals(new Plate(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/DiningTablewithChairsPlateCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameRefrigerator() {
		assertEquals(new Refrigerator(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/RefrigeratorCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameSink() {
		assertEquals(new Sink(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/SinksinkCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameSinkUnit() {
		assertEquals(new SinkUnit(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/SinkUnitCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameSpeaker() {
		assertEquals(new Speaker(),
				DataSet.extractReferenceFromName(new DataSet(
						"/test/file/TVspeakerCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameSpoon() {
		assertEquals(new Spoon(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/DiningTablewithChairsSpoonCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameTV() {
		assertEquals(new TV(), DataSet.extractReferenceFromName(new DataSet(
				"/test/file/TVCup.csv")));
	}

	@Test
	public void testExtractReferenceFromNameInvalidObject() {
		assertNull(DataSet.extractReferenceFromName(new DataSet(
				"/test/file/DiningTablewithChairsInvalidCup.csv")));
	}
}
