import json
import open3d as o3d
import matplotlib.pyplot as plt
#o3d.t.io.RealSenseSensor.list_devices() #lists if any devices are connected

bagfilename= 'intelcapture_1.bag' #bag file for storing the 3d Scans

#Define config for intelRealSense Camera
configgg= {
    "serial": "",
    "color_format": "RS2_FORMAT_RGB8",
    "color_resolution": "0,540",
    "depth_format": "RS2_FORMAT_Z16",
    "depth_resolution": "0,480",
    "fps": "30",
    "visual_preset": "RS2_L500_VISUAL_PRESET_MAX_RANGE"
 }


#initialize the sensor with config file
rs_cfg = o3d.t.io.RealSenseSensorConfig(configgg)
rs = o3d.t.io.RealSenseSensor()
rs.init_sensor(rs_cfg, 0) #initialize the sensor
rs.start_capture(True)  # true: start recording with capture

for fid in range(5):
    im_rgbd = rs.capture_frame(True, True)  # wait for frames and align them
    colorr= o3d.cpu.pybind.geometry.Image(im_rgbd.color)
    depthh= o3d.cpu.pybind.geometry.Image(im_rgbd.depth)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(colorr,depthh) #combine depth and color into an RGBD

    #convert the rgbd_image into a point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    #pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd]) #visualize the point cloud

    #use the code below to display depth and color image separately
    # plt.subplot(1, 2, 1)
    # plt.title('Color Image')
    # plt.imshow(rgbd_image.color)
    # plt.subplot(1, 2, 2)
    # plt.title('Depth Image')
    # plt.imshow(rgbd_image.depth)
    # plt.show()

rs.stop_capture()  #stops the capture