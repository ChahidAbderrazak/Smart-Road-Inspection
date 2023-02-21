def save_image_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame,scene_count, stop_threads, vid,  car_location
    # save image file locally
    filename_strg = str(get_sensor_filename("CSI_CAMERA", sensor_frame)) + ".jpg"
    image_save_path = os.path.join(data_root, "sweeps", "CSI_CAMERA", filename_strg)
    ret, frame = vid.read()
    cv2.imwrite(image_save_path, frame)
    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["scene"] = scene_count
    dict_frame["sensor_name"] = "CSI_CAMERA"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": []}
    dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"] = "jpg"
    dict_frame["filename"] = str(os.path.join("sweeps", "CSI_CAMERA", filename_strg))
    dict_frame["meta_data"] = ""