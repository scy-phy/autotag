# Additional Autoware Changes

Besides the modifications required by the original TUM Carla–Autoware Bridge, the following additional changes are required to reproduce our experimental setup.

## Summary

The modifications can be grouped into the following categories:

- Adaptations of the Carla T2 sensor kit configuration.
- Additional camera topic relays required by the perception pipeline.
- Launch configuration changes to use the Carla sensor kit and bridge topic layout. (from oribinal bridge)
- Configuration of the camera–LiDAR fusion pipeline for a single camera.
- Enabling the TensorRT YOLOX detector.
- Adjustments to ROI synchronization and perception parameters required for reliable camera–LiDAR fusion.
- Minor perception parameter changes.

---

# Required Changes

## `Carla_t2`

### `carla_t2_sensor_kit_launch/launch/camera.launch.xml`

Add the following relay nodes before the traffic light namespace.

```xml
<group>
  <push-ros-namespace namespace="front">

    <node pkg="topic_tools" exec="relay" name="front_camera_info_relay" output="log">
      <param name="input_topic" value="$(var camera_topic_name)/camera_info"/>
      <param name="output_topic" value="camera_info"/>
      <param name="type" value="sensor_msgs/msg/CameraInfo"/>
      <param name="reliability" value="best_effort"/>
    </node>

    <node pkg="topic_tools" exec="relay" name="front_image_relay" output="log">
      <param name="input_topic" value="$(var camera_topic_name)/image"/>
      <param name="output_topic" value="image_raw"/>
      <param name="type" value="sensor_msgs/msg/Image"/>
      <param name="reliability" value="best_effort"/>
    </node>

</group>
```

---

## `autoware_launch`

### `launch/autoware.launch.xml`

Change the sensor configuration directory to

```xml
<arg name="config_dir" value="$(find-pkg-share carla_t2_sensor_kit_description)/config/"/>
```

instead of

```xml
<arg name="config_dir" value="$(find-pkg-share individual_params)/config/$(var vehicle_id)/$(var sensor_model)"/>
```

---

### `launch/components/tier4_localization_component.launch.xml`

Change the LiDAR input topic to

```xml
<arg
  name="input_pointcloud"
  default="/sensor/lidar/front"
  description="The topic will be used in the localization util module"/>
```

and add

```xml
<arg
  name="lidar_container_name"
  default="/sensing/lidar/front/pointcloud_preprocessor/pointcloud_container"
  description="The target container to which lidar preprocessing nodes in localization be attached"/>
```

---

### `config/perception/object_recognition/detection/image_projection_based_fusion/roi_sync.param.yaml`

Replace

```yaml
input_offset_ms: [61.67, 111.67, 45.0, 28.33, 78.33, 95.0]
```

with

```yaml
input_offset_ms: [61.67]
```

and additionally set

```yaml
debug_mode: true
image_buffer_size: 15
```

---

## `autoware.universe`

### `launch/tier4_perception_launch/launch/object_recognition/detection/camera_lidar_fusion_based_detection.launch.xml`

Enable the TensorRT YOLOX detector by uncommenting the include and replacing

```xml
tensorrt_yolo
```

with

```xml
tensorrt_yolox
```

---

### `launch/tier4_perception_launch/launch/perception.launch.xml`

Change the first camera topic from

```xml
/sensing/camera/camera0/image_rect_color
```

to

```xml
/sensing/camera/camera0/image_raw
```

and change

```xml
<arg name="image_number" default="6"/>
```

to

```xml
<arg name="image_number" default="1"/>
```

---

### `perception/image_projection_based_fusion/launch/roi_cluster_fusion.launch.xml`

Configure the fusion pipeline to use a single camera:

```xml
<arg name="input/rois_number" default="1"/>
```

instead of

```xml
<arg name="input/rois_number" default="6"/>
```

---

### `perception/tensorrt_yolox/launch/yolox_s_plus_opt.launch.xml`

Change the image input topic to

```xml
<arg
  name="input/image"
  default="/sensing/camera/camera0/image_raw"/>
```

instead of

```xml
/sensing/camera/camera0/image_rect_color
```

and remap the output directly to

```xml
<remap
  from="~/out/objects"
  to="/perception/object_recognition/detection/rois0"/>
```

instead of

```xml
<remap
  from="~/out/objects"
  to="$(var output/objects)"/>
```