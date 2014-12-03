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

public class InteractiveWorldLearner {

	public static final String NODE_NAME = "interactive_world_learner";

	public static final String LEARN_HYPOTHESES_SERVICE_NAME = "/" + InteractiveWorldLearner.NODE_NAME + "/learn_hypotheses";

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
			Model[] models = new Model[placementSets.length];
			for (int i=0; i<placementSets.length; i++) {
				// create a data set
				PlacementSet placementSet = placementSets[i];
				Item item = placementSet.getItem();
				Room room = placementSet.getRoom();
				Surface surface = placementSet.getSurface();
				String referenceFrame = placementSet.getReferenceFrameId();
				DataSet ds = new DataSet(item, room, surface, referenceFrame);
				Placement[] placements = placementSet.getPlacements();
				for (int j=0; j<placements.length; j++) {
					Placement p = placements[j];
					Point position = p.getPosition();
					ds.add(position.getX(), position.getY(), position.getY(), p.getRotation());
				}

				// create the clustering model
				EMModel em = new EMModel(ds);
				models[i] = new Model(em.getPlacementLocation(), em.getDecisionValue(), 0, 0, 0, 0);
			}

			System.out.println("Model learning done, returning result.");
			// send back the response
			TaskModels taskModels = new TaskModels(models);
			LearnModels.Response resp = new LearnModels.Response(taskModels, true);
			learnHypotheses.sendResponse(resp, request.getId());
		}
	}

	public static void main(String[] args) {
		// create the learner object
		InteractiveWorldLearner l = new InteractiveWorldLearner();
		InteractiveWorldLearner.spin();
	}
}
