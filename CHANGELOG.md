# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## master -
### Added
- Method `getMonocularMatrix()` to get the intrinsic camera matrix of the monocular.
- Method `getStereoLink()` to get the link between the stereo camera and the base frame.

## 1.0.2 - 2020-10-22
### Changed
- Remove compiler warnings.

## 1.0.1 - 2020-10-19
### Changed
- Store the correct inverse of the respective poses.

## 1.0.0 - 2020-10-06
### Changed
- Return the inverse of the pose stored in the Link of the stereo node as a result of workspace calibration.

## 0.4.0 - 2020-09-01
### Changed
- Get rid of NxLibException calls and use the return value to wrap the API Calls in estd::result.
- Created two function to construct an ensenso object, one of them creating a shared_ptr object. Default constructor hidden.
- Improved overall error messages and handling.
- Expose "Fixed", useful for calibrating systems with lesser than 6 DOFs.

## 0.3.3 - 2020-07-30
### Changed
- Propagate the nxTree changes from the latest ensenso version update (2.3.1586).

## 0.3.2 - 2020-06-02
### Changed
- Link with --as-needed.
- Add Apache v2.0 license file.

## 0.3.1 - 2020-04-30
### Added
- ROI function for images and pointclouds in buffer.

## 0.3.0 - 2020-03-31
### Added
- ROI function to loadRegisteredPointCloud.
- Functions to write pointclouds to a buffer rather than a PCL pointcloud.
