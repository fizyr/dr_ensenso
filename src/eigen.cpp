#include "eigen.hpp"
#include "util.hpp"

namespace dr {

Result<Eigen::Vector3d> toEigenVector(NxLibItem const & item) {
	Result<double> item_0 = getNx<double>(item[0]);
	if (!item_0) return item_0.error();

	Result<double> item_1 = getNx<double>(item[1]);
	if (!item_1) return item_1.error();

	Result<double> item_2 = getNx<double>(item[2]);
	if (!item_2) return item_2.error();

	return Eigen::Vector3d{*item_0, *item_1, *item_2};
}

Result<Eigen::Matrix3d> toEigenMatrix(NxLibItem const & item) {
	auto column_0 = toEigenVector(item[0]);
	if (!column_0) return column_0.error();

	auto column_1 = toEigenVector(item[1]);
	if (!column_1) return column_1.error();

	auto column_2 = toEigenVector(item[2]);
	if (!column_2) return column_2.error();

	Eigen::Matrix3d result;
	result.col(0) = *column_0;
	result.col(1) = *column_1;
	result.col(2) = *column_2;
	return result;
}

Result<Eigen::Translation3d> toEigenTranslation(NxLibItem const & item) {
	Result<Eigen::Vector3d> translation = toEigenVector(item);
	if (!translation) return translation.error().push_description("failed to retrieve translation");

	return Eigen::Translation3d{*translation};
}

Result<Eigen::AngleAxisd> toEigenRotation(NxLibItem const & item) {
	Result<Eigen::Vector3d> axis = toEigenVector(item[itmAxis]);
	if (!axis) return axis.error().push_description("failed to retrieve rotation");

	Result<double> angle = getNx<double>(item[itmAngle]);
	if (!angle) return angle.error().push_description("failed to retrieve rotation");

	return Eigen::AngleAxisd{*angle, *axis};
}

Result<Eigen::Isometry3d> toEigenIsometry(NxLibItem const & item) {
	Result<Eigen::Translation3d> eigen_translation = toEigenTranslation(item[itmTranslation]);
	if (!eigen_translation) return eigen_translation.error().push_description("failed to retrieve isometry");

	Result<Eigen::AngleAxisd> eigen_rotation = toEigenRotation(item[itmRotation]);
	if (!eigen_rotation) return eigen_rotation.error().push_description("failed to retrieve isometry");


	return *eigen_translation * *eigen_rotation;
}

Result<void> setNx(NxLibItem const & item, Eigen::Vector3d const & vector) {
	Result<void> set_item_0 = setNx(item[0], vector.x());
	if (!set_item_0) return set_item_0.error();

	Result<void> set_item_1 = setNx(item[1], vector.y());
	if (!set_item_1) return set_item_1.error();

	Result<void> set_item_2 = setNx(item[2], vector.z());
	if (!set_item_2) return set_item_2.error();

	return estd::in_place_valid;
}

Result<void> setNx(NxLibItem const & item, Eigen::Translation3d const & translation) {
	return setNx(item, translation.vector());
}

Result<void> setNx(NxLibItem const & item, Eigen::AngleAxisd const & rotation) {
	Result<void> set_nx_angle = setNx(item[itmAngle], rotation.angle());
	if (!set_nx_angle) return set_nx_angle.error().push_description("failed to set rotation");

	Result<void> set_nx_axis = setNx(item[itmAxis], rotation.axis());
	if (!set_nx_axis) return set_nx_axis.error().push_description("failed to set rotation");

	return estd::in_place_valid;
}

Result<void> setNx(NxLibItem const & item, Eigen::Isometry3d const & isometry) {
	Result<void> set_nx_translation = setNx(item[itmTranslation], Eigen::Vector3d{isometry.translation()});
	if (!set_nx_translation) return set_nx_translation.error().push_description("failed to set isometry");

	Result<void> set_nx_rotation = setNx(item[itmRotation], Eigen::AngleAxisd{isometry.rotation()});
	if (!set_nx_rotation) return set_nx_rotation.error().push_description("failed to set isometry");

	return estd::in_place_valid;
}

}
