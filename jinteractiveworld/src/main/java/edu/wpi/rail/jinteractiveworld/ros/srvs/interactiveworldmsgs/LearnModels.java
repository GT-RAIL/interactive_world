package edu.wpi.rail.jinteractiveworld.ros.srvs.interactiveworldmsgs;

import javax.json.Json;
import javax.json.JsonObject;

import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.TaskModels;
import edu.wpi.rail.jinteractiveworld.ros.msgs.interactiveworldmsgs.TaskTrainingData;
import edu.wpi.rail.jrosbridge.services.ServiceRequest;
import edu.wpi.rail.jrosbridge.services.ServiceResponse;

import java.io.StringReader;
import java.util.Arrays;

/**
 * The interactive_world_msgs/LearnModels service.
 *
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 3, 2014
 */
public class LearnModels {

    /**
     * The service type.
     */
    public static final String TYPE = "interactive_world_msgs/LearnModels";

    /**
     * The service request for the interactive_world_msgs/LearnModels service.
     *
     * @author Russell Toris -- rctoris@wpi.edu
     * @version December 3, 2014
     */
    public static class Request extends ServiceRequest {

        /**
         * The name of the data field for the request.
         */
        public static final String FIELD_DATA = "data";

        private final TaskTrainingData data;

        /**
         * Create a new empty LearnModels ServiceRequest.
         */
        public Request() {
            this(new TaskTrainingData());
        }

        /**
         * Create a new LearnModels ServiceRequest.
         *
         * @param data
         *            The data of the request.
         */
        public Request(TaskTrainingData data) {
            super(Json.createObjectBuilder().add(Request.FIELD_DATA, data.toJsonObject()).build(), LearnModels.TYPE);
            this.data = data;
        }

        /**
         * Create a clone of this LearnModels ServiceRequest.
         */
        @Override
        public Request clone() {
            return new LearnModels.Request(this.data);
        }

        /**
         * Create a new LearnModels ServiceRequest based on the given JSON string. Any
         * missing values will be set to their defaults.
         *
         * @param jsonString
         *            The JSON string to parse.
         * @return A LearnModels ServiceRequest based on the given JSON string.
         */
        public static Request fromJsonString(String jsonString) {
            // convert to a ServiceRequest
            return LearnModels.Request.fromServiceRequest(new ServiceRequest(
                    jsonString));
        }

        /**
         * Create a new LearnModels ServiceRequest based on the given ServiceRequest.
         * Any missing values will be set to their defaults.
         *
         * @param req
         *            The ServiceRequest to parse.
         * @return A LearnModels ServiceRequest based on the given Message.
         */
        public static Request fromServiceRequest(ServiceRequest req) {
            // get it from the JSON object
            return LearnModels.Request.fromJsonObject(req.toJsonObject());
        }

        /**
         * Create a new LearnModels ServiceRequest based on the given JSON object. Any
         * missing values will be set to their defaults.
         *
         * @param jsonObject
         *            The JSON object to parse.
         * @return A LearnModels ServiceRequest based on the given JSON object.
         */
        public static Request fromJsonObject(JsonObject jsonObject) {
            TaskTrainingData data = jsonObject.containsKey(Request.FIELD_DATA) ? TaskTrainingData
                    .fromJsonObject(jsonObject.getJsonObject(Request.FIELD_DATA))
                    : new TaskTrainingData();
            return new LearnModels.Request(data);
        }
    }

    /**
     * The service response for the interactive_world_msgs/LearnModels service.
     *
     * @author Russell Toris -- rctoris@wpi.edu
     * @version December 3, 2014
     */
    public static class Response extends ServiceResponse {

        /**
         * The name of the models field for the request.
         */
        public static final String FIELD_MODELS = "models";

        private final TaskModels models;

        /**
         * Create a new empty LearnModels ServiceResponse.
         */
        public Response() {
            this(new TaskModels(), true);
        }

        /**
         * Create a new LearnModels ServiceResponse.
         *
         * @param models
         *            The models of the response.
         * @param result
         *            The result flag for the response (i.e., if the service
         *            server returned a success).
         */
        public Response(TaskModels models, boolean result) {
            super(Json.createObjectBuilder().add(Response.FIELD_MODELS, models.toJsonObject()).build(), LearnModels.TYPE, result);
            this.models = models;
        }

        /**
         * Create a clone of this LearnModels ServiceResponse.
         */
        @Override
        public Response clone() {
            return new LearnModels.Response(this.models, this.getResult());
        }

        /**
         * Create a new LearnModels ServiceResponse based on the given JSON string.
         * Any missing values will be set to their defaults.
         *
         * @param jsonString
         *            The JSON string to parse.
         * @return A LearnModels ServiceResponse based on the given JSON string.
         */
        public static Response fromJsonString(String jsonString) {
            // convert to a ServiceResponse
            return LearnModels.Response.fromServiceResponse(new ServiceResponse(
                    jsonString, true));
        }

        /**
         * Create a new LearnModels ServiceResponse based on the given
         * ServiceResponse. Any missing values will be set to their defaults.
         *
         * @param resp
         *            The ServiceResponse to parse.
         * @return A LearnModels ServiceResponse based on the given Message.
         */
        public static Response fromServiceResponse(ServiceResponse resp) {
            // get it from the JSON object
            return LearnModels.Response.fromJsonObject(resp.toJsonObject());
        }

        /**
         * Create a new LearnModels ServiceResponse based on the given JSON object.
         * Any missing values will be set to their defaults.
         *
         * @param jsonObject
         *            The JSON object to parse.
         * @return A LearnModels ServiceResponse based on the given JSON object.
         */
        public static Response fromJsonObject(JsonObject jsonObject) {
            TaskModels models = jsonObject.containsKey(Response.FIELD_MODELS) ? TaskModels
                    .fromJsonObject(jsonObject.getJsonObject(Response.FIELD_MODELS))
                    : new TaskModels();
            return new LearnModels.Response(models, true);
        }
    }
}
