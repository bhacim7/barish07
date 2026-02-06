import pyzed.sl as sl
import time
import sys
#FIXME: BU KODU GİT'TE GÜNCELLE
# Basic class to handle the timestamp of the different sensors to know if it is a new sensors_data or an old one
class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()

    # check if the new timestamp is higher than the reference one, and if yes, save the current as reference
    def is_new(self, sensor):
        if isinstance(sensor, sl.IMUData):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_


def initialize_camera():
    # ZED kamera nesnesi oluştur
    zed = sl.Camera()
    # ZED başlatma parametreleri ayarla
    init_params = sl.InitParameters()
    runtime_params = sl.RuntimeParameters(enable_fill_mode=True)
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 720p çözünürlük sl.RESOLUTION.VGA geç fps için
    init_params.camera_fps = 30  # 30 FPS
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE #todo: VİDEO İÇİN EN DÜŞÜĞE ALINACAK. depth mode best quality at neural_plus
    init_params.coordinate_units = sl.UNIT.METER  #using metric system
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP  # finale
    init_params.depth_minimum_distance = 0.20
    init_params.depth_maximum_distance = 20
    init_params.camera_disable_self_calib = False
    init_params.depth_stabilization = 50  # titreme azaltıcı
    init_params.sensors_required = False  # true yaparsan imu açılmadan kamera açılmaz
    init_params.enable_image_enhancement = True  # true was always the default
    init_params.async_grab_camera_recovery = False  # set true if u want to keep processing if cam gets shutdown
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        raise Exception("Failed to open ZED camera. Exiting.")
    return zed


# Initialize positional tracking
def initialize_positional_tracking(zed):
    # Enable positional tracking with default parameters
    py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
    tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
    tracking_parameters.enable_pose_smoothing = True  # set true to lower the loop_closures. effect Default False
    tracking_parameters.set_floor_as_origin = False  # Set the floor as the reference point Default False todo: True ile dene
    tracking_parameters.enable_area_memory = True  # Persistent memory for localization, Whether the camera can remember its surroundings. Default True
    tracking_parameters.enable_imu_fusion = True  # When set to False, only the optical odometry will be used. Default True
    tracking_parameters.set_as_static = False  # Whether to define the camera as static. Default False
    tracking_parameters.depth_min_range = 0.40  # It may be useful for example if any steady objects are in front of the camera and may perturb the positional tracking algorithm. Dfault no min range: -1
    tracking_parameters.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1  # gen_1 for performance, gen_2 for accuracy #todo: GEN_1 ve GEN_2 kıyaslamasını yap.

    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Enable positional tracking : " + repr(err) + ". Exit program.")
        zed.close()
        exit()


# Initialize spatial mapping
def initialize_spatial_mapping(zed):
    # Enable spatial mapping
    mapping_parameters = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.MESH)  # .mesh or .fused_point_cloud
    mapping_parameters.resolution_meter = 0.10  # Define resolution (0.05m for fine mapping)
    mapping_parameters.range_meter = 20  # Maximum range for the mesh
    mapping_parameters.use_chunk_only = True  # Allow chunks of the map #true for better performance #false for accuracy
    mapping_parameters.max_memory_usage = 2048  # The maximum CPU memory (in MB) allocated for the meshing process. #default 2048
    # Whether to inverse the order of the vertices of the triangles.If your display process does not handle front and back face culling,
    mapping_parameters.reverse_vertex_order = False  # you can use this to correct it. Default: False. only for mesh

    error = zed.enable_spatial_mapping(mapping_parameters)
    if error != sl.ERROR_CODE.SUCCESS:
        raise Exception(f"Spatial mapping initialization failed: {error}")

def kapat(cam: sl.Camera):
    # Hangi modüller açıksa kapatmayı dene (açıksa kapanır, değilse sessiz geçer)
    for fn in ("disable_recording", "disable_spatial_mapping", "disable_positional_tracking"):
        try:
            getattr(cam, fn)()
        except Exception:
            pass
    try:
        cam.close()
    except Exception:
        pass

def temiz_kapat(cam: sl.Camera):
    # Açık modüller varsa sessizce kapat
    for fn in ("disable_recording", "disable_spatial_mapping", "disable_positional_tracking"):
        try:
            getattr(cam, fn)()
        except Exception:
            pass
    try:
        cam.close()
    except Exception:
        pass