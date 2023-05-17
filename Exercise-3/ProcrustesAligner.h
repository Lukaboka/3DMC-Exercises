#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements

		Matrix4f estimatedPose = Matrix4f::Identity();

		/*
		* r_x 0   0   t_x
		* 0   r_y 0   t_y
		* 0   0   r_z t_z
		* 0   0   0   1
		*/
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;

		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.
		Vector3f mean = Vector3f::Zero();

		for (int i = 0; i < points.size(); i++)
		{
			mean += points[i];
		}

		return mean / points.size();
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).

		// X = target object point cloud, X_hat = moved object point cloud
		MatrixXf X = MatrixXf(3, sourcePoints.size());
		MatrixXf X_hat = MatrixXf(3, targetPoints.size());

		// create matrix X and X_hat from point vectors
		for (int i = 0; i < sourcePoints.size(); i++)
		{
			X.block(0, i, 3, 1) = sourcePoints[i] - sourceMean;
			X_hat.block(0, i, 3, 1) = targetPoints[i] - targetMean;
		}

		Matrix3f A = X * X_hat.transpose();

		// SVD of X * X_hat
		MatrixXf U = A.jacobiSvd(ComputeFullU);
		MatrixXf V = A.jacobiSvd(ComputeFullV);

		Matrix3f rotation;

		// if det(U*V^T) == -1 mirror the rotation matrix 
		if ((U * V.transpose()).determinant() == -1)
		{
			Matrix3f m;
			m << 1, 0, 0,
				0, 1, 0,
				0, 0, -1;
			rotation = U * m * V.transpose();
		}
		else
		{
			rotation = U * V.transpose();
		}

        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.

		Vector3f translation = - rotation * sourceMean + targetMean;
        return translation;
	}
};

