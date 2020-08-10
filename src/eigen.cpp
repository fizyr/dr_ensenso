#include "eigen.hpp"
#include "util.hpp"

namespace dr {

Result<Eigen::Vector3d> toEigenVector(NxLibItem const & item) {
	Result<double> get_nx_item_0_result = getNx<double>(item[0]);
	if (!get_nx_item_0_result) return get_nx_item_0_result.error();

	Result<double> get_nx_item_1_result = getNx<double>(item[1]);
	if (!get_nx_item_1_result) return get_nx_item_1_result.error();

	Result<double> get_nx_item_2_result = getNx<double>(item[2]);
	if (!get_nx_item_2_result) return get_nx_item_2_result.error();

	return Eigen::Vector3d{*get_nx_item_0_result, *get_nx_item_1_result, *get_nx_item_2_result};
}

Result<Eigen::Translation3d> toEigenTranslation(NxLibItem const & item) {
	Result<Eigen::Vector3d> to_eigen_vector_result = toEigenVector(item);
	if (!to_eigen_vector_result) return to_eigen_vector_result.error();

	return Eigen::Translation3d{*to_eigen_vector_result};
}

Result<Eigen::AngleAxisd> toEigenRotation(NxLibItem const & item) {
	Result<Eigen::Vector3d> to_eigen_vector_result = toEigenVector(item[itmAxis]);
	if (!to_eigen_vector_result) return to_eigen_vector_result.error();

	Result<double> get_nx_item_angle_result = getNx<double>(item[itmAngle]);
	if (!get_nx_item_angle_result) return get_nx_item_angle_result.error();

	return Eigen::AngleAxisd{*get_nx_item_angle_result, *to_eigen_vector_result};
}

Result<Eigen::Isometry3d> toEigenIsometry(NxLibItem const & item) {
	Result<Eigen::Translation3d> to_eigen_translation_result = toEigenTranslation(item[itmTranslation]);
	if (!to_eigen_translation_result) return to_eigen_translation_result.error();

	Result<Eigen::AngleAxisd> to_eigen_rotation_result = toEigenRotation(item[itmRotation]);
	if (!to_eigen_rotation_result) return to_eigen_rotation_result.error();


	return *to_eigen_translation_result * *to_eigen_rotation_result;
}

Result<void> setNx(NxLibItem const & item, Eigen::Vector3d const & vector, std::string const & what) {
	Result<void> set_nx_item_0_result = setNx(item[0], vector.x(), what);
	if (!set_nx_item_0_result) return set_nx_item_0_result.error();

	Result<void> set_nx_item_1_result = setNx(item[1], vector.y(), what);
	if (!set_nx_item_1_result) return set_nx_item_1_result.error();

	Result<void> set_nx_item_2_result = setNx(item[2], vector.z(), what);
	if (!set_nx_item_2_result) return set_nx_item_2_result.error();

	return estd::in_place_valid;
}

Result<void> setNx(NxLibItem const & item, Eigen::Translation3d const & translation, std::string const & what) {
	Result<void> set_nx_translation_result = setNx(item, translation.vector(), what);
	if (!set_nx_translation_result) return set_nx_translation_result.error();

	return estd::in_place_valid;
}

Result<void> setNx(NxLibItem const & item, Eigen::AngleAxisd const & rotation, std::string const & what) {
	Result<void> set_nx_angle_result = setNx(item[itmAngle], rotation.angle(), what);
	if (!set_nx_angle_result) return set_nx_angle_result.error();

	Result<void> set_nx_axis_result = setNx(item[itmAxis], rotation.axis(), what);
	if (!set_nx_axis_result) return set_nx_axis_result.error();

	return estd::in_place_valid;
}

Result<void> setNx(NxLibItem const & item, Eigen::Isometry3d const & isometry, std::string const & what) {
	Result<void> set_nx_translation_result = setNx(item[itmTranslation], Eigen::Vector3d{isometry.translation()}, what);
	if (!set_nx_translation_result) return set_nx_translation_result.error();

	Result<void> set_nx_rotation_result = setNx(item[itmRotation], Eigen::AngleAxisd{isometry.rotation()}, what);
	if (!set_nx_rotation_result) return set_nx_rotation_result.error();

	return estd::in_place_valid;
}

}
