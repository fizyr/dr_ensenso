#pragma once
#include <Eigen/Dense>
#include <ensenso/nxLib.h>
#include <cstdint>
#include "util.hpp"

namespace dr {

/// Convert an NxLibItem holding a 3D vector to an Eigen::Vector3d.
Eigen::Vector3d toEigenVector(NxLibItem const & item);

/// Convert an NxLibItem holding a 3D vector to an Eigen::Translation3d.
Eigen::Translation3d toEigenTranslation(NxLibItem const & item);

/// Convert an NxLibItem holding a rotation to an Eigen::AngleAxisd.
Eigen::AngleAxisd toEigenRotation(NxLibItem const & item);

/// Convert an NxLibItem holding a pose or transformation to an Eigen::Isometry3d.
Eigen::Isometry3d toEigenIsometry(NxLibItem const & item);

/// Set an Eigen::Vector3d in an NxLibItem.
void setNx(NxLibItem const & item, Eigen::Vector3d const & vector, std::string const & what = "");

/// Set an Eigen::Vector3d in an NxLibItem.
void setNx(NxLibItem const & item, Eigen::Translation3d const & translation, std::string const & what = "");

/// Set an Eigen::Vector3d in an NxLibItem.
void setNx(NxLibItem const & item, Eigen::AngleAxisd const & rotation, std::string const & what = "");

/// Set an Eigen::Vector3d in an NxLibItem.
void setNx(NxLibItem const & item, Eigen::Isometry3d const & isometry, std::string const & what = "");

template <std::size_t rows, std::size_t cols>
Eigen::Matrix<double, rows, cols> toEigenMatrix(NxLibItem const & item) {
	Eigen::Matrix<double, rows, cols> result;

	for (std::size_t row = 0; row < rows; ++row) {
		for (std::size_t col = 0; col < cols; ++col) {
			result(row, col) = getNx<double>(item[col][row]);
		}
	}

	return result;
}

}
