# Third-Party Notices

This repository contains or integrates research software with independent
licenses and model terms. Their original notices remain authoritative.

- <code>baselines/navdp/</code> and the benchmark root derive from
  [InternRobotics/NavDP](https://github.com/InternRobotics/NavDP), whose README
  states CC BY-NC-SA 4.0 terms for its open-source code.
- <code>baselines/x-navdp/</code> retains its included MIT
  <code>LICENSE</code> and <code>CITATION.cff</code>.
- <code>baselines/navdp/depth_anything/</code> and
  <code>baselines/x-navdp/third_party/depth_anything/</code> retain the bundled
  Depth Anything notices.
- The physical bridge uses Unitree SDK2 Python and CycloneDDS supplied by the
  operator; neither is redistributed here.
- The camera path uses Intel librealsense and realsense-ros supplied by the
  operator.
- The optional independent reference lane builds the official
  [Manifold Odin ROS Driver](https://github.com/manifoldsdk/odin_ros_driver)
  at a pinned commit. The driver and its binary SDK remain under their upstream
  license/terms and are not vendored. The default 0.14 profile carries only a
  ROS2 runtime-config fix; the separate 0.13.1 Mode1 patch is retained solely
  as an auditable historical compatibility option.
- The optional certified revisit path depends on external LingBot Map,
  LightGlue, InternNav and MemNav research source. This repository does not
  grant additional rights to those projects.

Model weights and datasets are not distributed. Obtain every artifact from
its owner and follow its license, access and attribution requirements.
