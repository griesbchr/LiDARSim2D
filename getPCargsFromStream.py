#import SensorView

def getpcargsfromstream(model):
    '''
    This is a pseudo code function that extracts the information needed to calculate the lidar pointcloud
    via the sensor models from an OSI stream.
    :param model: sensor model used to calculate the pointcloud
    :return: args: tuple of arguments which can be handed to the getPointcloud functions
    '''

    #Initialize SensorView and SensorData
    #sv = SensorView()

    #Get attributes from SensorView
    ego_x_y_th = 0  #from sv.host_vehicle_data.location.position and sv.host_vehicle_data.location.orientation
    scene_vertices = 0 #from sv.global_ground_truth.stationary_object.base.base_polygon
    scene_line_vecs = 0 #calc from scene_vertices
    target_line_vecs = 0 #from sv.global_ground_truth.moving_object.base.base_polygon
    target_vertices = 0 #calc from target_line_vecs
    n_rays = 0 #from sv.lidar_sensor_view.view_configuration.number_of_rays_horizontal
    alpha_init = 0 #from sv.lidar_sensor_view.view_configuration.directions/mounting_position.orientation
    fov = 0 #from sv.lidar_sensor_view.view_configuration.field_of_view_horizontal
    alpha_inc = 0 #calc from fov and n_rays
    lidarmountx = 0 #from sv.lidar_sensor_view.view_configuration.mounting_position.position.x
    lidarmounty = 0 #from sv.lidar_sensor_view.view_configuration.mounting_position.position.y
    counterclockwise = 0 #for fov<360: from a wrapper function that keeps track of the current lidar moving direction, for fov=360: sensor data sheet?
    d_max = 0  #sensor data sheet?

    #Get model spesific attributes from SensorView
    if model == "realsensor":

        PC_update = 0 #from wrapper function
        current_alpha = 0 #from sv.lidar_sensor_view.view_configuration.directions?


        args = (ego_x_y_th,
                scene_vertices, scene_line_vecs, target_line_vecs, target_vertices,
                d_max, lidarmountx, lidarmounty,
                current_alpha, PC_update)
        return args

    if model == "model0":

        args = (ego_x_y_th,
                scene_vertices, scene_line_vecs, target_vertices, target_line_vecs,
                n_rays, alpha_init, fov, alpha_inc,
                lidarmountx, lidarmounty, d_max)

    if model == "model1":

        ego_x_y_th_prev = 0 #from sv.host_vehicle_data.location.position and sv.host_vehicle_data.location.orientation

        args = (ego_x_y_th, ego_x_y_th_prev,
                scene_vertices, scene_line_vecs, target_vertices, target_line_vecs,
                n_rays, alpha_init, fov, alpha_inc,
                lidarmountx, lidarmounty,
                counterclockwise, d_max)

    if model == "model2":

        ego_x_y_th_prev = 0 #from sv.host_vehicle_data.location.position and sv.host_vehicle_data.location.orientation
        target_x_y_th = 0 #from sv. global_ground_truth.moving_object.base.position and v. global_ground_truth.moving_object.base.orientation
        target_x_y_th_prev = 0 #from sv. global_ground_truth.moving_object.base.position and v. global_ground_truth.moving_object.base.orientation

        lowerleft = 0 #calculate from sv. global_ground_truth.moving_object.base.base_polygon
        lowerright = 0 #calculate from sv. global_ground_truth.moving_object.base.base_polygon
        upperleft = 0 #calculate from sv. global_ground_truth.moving_object.base.base_polygon
        upperright = 0 #calculate from sv. global_ground_truth.moving_object.base.base_polygon

        args = (ego_x_y_th, target_x_y_th, ego_x_y_th_prev, target_x_y_th_prev,
                lowerleft, lowerright, upperleft, upperright,
                scene_vertices, scene_line_vecs,
                n_rays, alpha_init, fov, alpha_inc,
                lidarmountx, lidarmounty,
                counterclockwise, d_max)

    if model == "model3":

        ego_x_y_th_prev = 0 #from sv.host_vehicle_data.location.position and sv.host_vehicle_data.location.orientation
        target_vertices_prev = 0 #calculate from sv.global_ground_truth.moving_object.base.base_polygon
        target_line_vecs_prev = 0  #calculate from target_vertices_prev


        args = (ego_x_y_th, ego_x_y_th_prev,
                scene_vertices, scene_line_vecs,
                target_vertices, target_line_vecs, target_vertices_prev, target_line_vecs_prev,
                n_rays, alpha_init, fov, alpha_inc,
                lidarmountx, lidarmounty,
                counterclockwise, d_max)

    return args