#pragma once
#include <Eigen/Dense>
#include <ensenso/nxLib.h>
#include <cstdint>
#include "util.hpp"

namespace dr {

/// Convert an NxLibItem holding a 3D vector to an Eigen::Vector3d.
Result<Eigen::Vector3d> toEigenVector(NxLibItem const & item);

/// Convert an NxLibItem holding a 3x3 matrix to an Eigen::Matrix3d.
Result<Eigen::Matrix3d> toEigenMatrix(NxLibItem const & item);

/// Convert an NxLibItem holding a 3D vector to an Eigen::Translation3d.
Result<Eigen::Translation3d> toEigenTranslation(NxLibItem const & item);

/// Convert an NxLibItem holding a rotation to an Eigen::AngleAxisd.
Result<Eigen::AngleAxisd> toEigenRotation(NxLibItem const & item);

/// Convert an NxLibItem holding a pose or transformation to an Eigen::Isometry3d.
Result<Eigen::Isometry3d> toEigenIsometry(NxLibItem const & item);

/// Set an Eigen::Vector3d in an NxLibItem.
Result<void> setNx(NxLibItem const & item, Eigen::Vector3d const & vector);

/// Set an Eigen::Vector3d in an NxLibItem.
Result<void> setNx(NxLibItem const & item, Eigen::Translation3d const & translation);

/// Set an Eigen::Vector3d in an NxLibItem.
Result<void> setNx(NxLibItem const & item, Eigen::AngleAxisd const & rotation);

/// Set an Eigen::Vector3d in an NxLibItem.
Result<void> setNx(NxLibItem const & item, Eigen::Isometry3d const & isometry);

template <std::size_t rows, std::size_t cols>
Result<Eigen::Matrix<double, rows, cols>> toEigenMatrix(NxLibItem const & item) {
	Eigen::Matrix<double, rows, cols> result;

	for (std::size_t row = 0; row < rows; ++row) {
		for (std::size_t col = 0; col < cols; ++col) {
			Result<double> value = getNx<double>(item[col][row]);
			if (!value) {
				return value.error().push_description("failed to retrieve matrix: ");
			}
			result(row, col) = *value;
		}
	}

	return result;
}

}
