package edu.wpi.rail.jinteractiveworld.ros;

import edu.wpi.rail.jinteractiveworld.data.DataSet;
import edu.wpi.rail.jinteractiveworld.model.EMModel;
import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.*;
import edu.wpi.rail.jinteractiveworld.ros.srvs.interactiveworldmsgs.LearnModels;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Service;
import edu.wpi.rail.jrosbridge.callback.CallServiceCallback;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;
import java.sql.*;
import java.util.ArrayList;

public class InteractiveWorldLearner {

	public static final String NODE_NAME = "interactive_world_learner";

	public static final String LEARN_HYPOTHESES_SERVICE_NAME = "/" + InteractiveWorldLearner.NODE_NAME + "/learn_hypotheses";

	public static final String DB_CLASS = "com.mysql.jdbc.Driver";

	public static final String DB_CREATE = "CREATE TABLE IF NOT EXISTS `iwmodels` (" +
			"  `id` int(10) unsigned NOT NULL AUTO_INCREMENT," +
			"  `condition_id` int(10) unsigned NOT NULL," +
			"  `value` mediumtext NOT NULL," +
			"  `created` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP," +
			"  `modified` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00'," +
			"  PRIMARY KEY (`id`)," +
			"  UNIQUE KEY `condition_id` (`condition_id`)" +
			") ENGINE=InnoDB DEFAULT CHARSET=latin1 AUTO_INCREMENT=1 ;";

	public static final String DB_FOREIGN_KEY = "ALTER TABLE `iwmodels`" +
			"  ADD CONSTRAINT `iwmodels_ibfk_1` FOREIGN KEY (`condition_id`) REFERENCES `conditions` (`id`) ON DELETE CASCADE ON UPDATE CASCADE;";

	public static final String DB_ENTRY_CHECK = "SELECT `id` FROM `iwmodels` WHERE `condition_id`=?";

	public static final String DB_ENTRY_DELETE = "DELETE FROM `iwmodels` WHERE `condition_id`=?";

	public static final String DB_INSERT_MODEL = "INSERT INTO `iwmodels` (`condition_id`, `value`) VALUES (?,?)";

	private Ros ros;
	private Service learnHypotheses;

	public InteractiveWorldLearner() {
		ros = new Ros();
		System.out.print("Attempting connection to rosbridge...");
		if (!ros.connect()) {
			System.out.println("No connection made. Verify rosbridge_websocket is running.");
			System.exit(-1);
		}
		System.out.println(" success!");

		// setup the services
		learnHypotheses = new Service(ros, InteractiveWorldLearner.LEARN_HYPOTHESES_SERVICE_NAME, "interactive_world_msgs/LearnModels");
		learnHypotheses.advertiseService(new LearnModelsCallback());
	}

	public static void spin() {
		while (true) {
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	private class LearnModelsCallback implements CallServiceCallback {

		@Override
		public void handleServiceCall(ServiceRequest request) {
			System.out.println("Model learning request received...");
			// parse the message
			LearnModels.Request req = LearnModels.Request.fromJsonObject(request.toJsonObject());

			// go trough each
			PlacementSet[] placementSets = req.getData().getData();
			ArrayList<Model> models = new ArrayList<Model>();
			for (int i=0; i<placementSets.length; i++) {
				// create a data set
				PlacementSet placementSet = placementSets[i];
				if (placementSet.getPlacements().length > 1) {
					Item item = placementSet.getItem();
					Room room = placementSet.getRoom();
					Surface surface = placementSet.getSurface();
					String referenceFrame = placementSet.getReferenceFrameId();
					DataSet ds = new DataSet(item, room, surface, referenceFrame);
					Placement[] placements = placementSet.getPlacements();
					for (int j = 0; j < placements.length; j++) {
						Placement p = placements[j];
						Point position = p.getPosition();
						ds.add(position.getX(), position.getY(), position.getY(), p.getRotation());
					}

					// create the clustering model
					EMModel em = new EMModel(ds, EMModel.RankingFunction.CUSTOM);
					models.add(new Model(em));
				}
			}

			Model[] modelsArray = new Model[models.size()];
			models.toArray(modelsArray);
			TaskModels taskModels = new TaskModels(modelsArray);
			System.out.println("Model learning done.");

			// store the models in the RMS
			System.out.println("Attempting connection to the RMS...");
			try {
				Class.forName(InteractiveWorldLearner.DB_CLASS);
				String url = "jdbc:mysql://" + req.getHost() + ":" + req.getPort() + "/" + req.getDatabase();
				Connection connection = DriverManager.getConnection(url, req.getUser(), req.getPassword());

				// check for the table
				DatabaseMetaData dbm = connection.getMetaData();
				ResultSet tables = dbm.getTables(null, null, "iwmodels", null);
				if (!tables.next()) {
					// create the table
					Statement create = connection.createStatement();
					create.executeUpdate(InteractiveWorldLearner.DB_CREATE);
					Statement foreignKey = connection.createStatement();
					foreignKey.executeUpdate(InteractiveWorldLearner.DB_FOREIGN_KEY);
				}

				// check if we should clear out an old entry
				final PreparedStatement check = connection.prepareStatement(InteractiveWorldLearner.DB_ENTRY_CHECK);
				final int i1 = 1;
				final int i2 = 2;
				check.setInt(i1, req.getConditionId());
				final ResultSet existing = check.executeQuery();
				if(existing.next()) {
					// delete the old one
					final PreparedStatement delete = connection.prepareStatement(InteractiveWorldLearner.DB_ENTRY_DELETE);
					delete.setInt(i1, req.getConditionId());
					delete.executeUpdate();
				}

				// create our entry
				final PreparedStatement insert = connection.prepareStatement(InteractiveWorldLearner.DB_INSERT_MODEL);
				insert.setInt(i1, req.getConditionId());
				insert.setString(i2, taskModels.toString());
				insert.executeUpdate();

				// close the connection
				connection.close();
				System.out.println("Models stored to the RMS.");
			} catch (ClassNotFoundException|SQLException e) {
				System.err.println("Could not connect the the RMS: " + e.getMessage());
			}

			// send back the response
			LearnModels.Response resp = new LearnModels.Response(true);
			learnHypotheses.sendResponse(resp, request.getId());
		}
	}

	public static void main(String[] args) {
		// create the learner object
		InteractiveWorldLearner l = new InteractiveWorldLearner();
		InteractiveWorldLearner.spin();
	}
}
