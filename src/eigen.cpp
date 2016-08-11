#include "eigen.hpp"
#include "util.hpp"

namespace dr {

Eigen::Vector3d toEigenVector(NxLibItem const & item) {
	return Eigen::Vector3d{getNx<double>(item[0]), getNx<double>(item[1]), getNx<double>(item[2])};
}

Eigen::Translation3d toEigenTranslation(NxLibItem const & item) {
	return Eigen::Translation3d{toEigenVector(item)};
}

Eigen::AngleAxisd toEigenRotation(NxLibItem const & item) {
	return Eigen::AngleAxisd{getNx<double>(item[itmAngle]), toEigenVector(item[itmAxis])};
}

Eigen::Isometry3d toEigenIsometry(NxLibItem const & item) {
	return toEigenTranslation(item[itmTranslation]) * toEigenRotation(item[itmRotation]);
}

void setNx(NxLibItem const & item, Eigen::Vector3d const & vector, std::string const & what) {
	setNx(item[0], vector.x(), what);
	setNx(item[1], vector.y(), what);
	setNx(item[2], vector.z(), what);
}

void setNx(NxLibItem const & item, Eigen::Translation3d const & translation, std::string const & what) {
	setNx(item, translation.vector(), what);
}

void setNx(NxLibItem const & item, Eigen::AngleAxisd const & rotation, std::string const & what) {
	setNx(item[itmAngle], rotation.angle(), what);
	setNx(item[itmAxis], rotation.axis(), what);
}

void setNx(NxLibItem const & item, Eigen::Isometry3d const & isometry, std::string const & what) {
	setNx(item[itmTranslation], Eigen::Vector3d{isometry.translation()}, what);
	setNx(item[itmRotation], Eigen::AngleAxisd{isometry.rotation()}, what);
}

}
