/*
 * Eos - A 3D Morphable Model fitting library written in modern C++11/14.
 *
 * File: include/eos/morphablemodel/PcaModel.hpp
 *
 * Copyright 2014, 2015 Patrik Huber
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <eos/morphablemodel/PcaModel.h>

namespace eos {
	namespace morphablemodel {

// Forward declarations of free functions
cv::Mat normalise_pca_basis(cv::Mat unnormalisedBasis, cv::Mat eigenvalues);
cv::Mat unnormalise_pca_basis(cv::Mat normalisedBasis, cv::Mat eigenvalues);

/**
 * @brief This class represents a PCA-model that consists of:
 *   - a mean vector (y x z)
 *   - a PCA basis matrix (unnormalised and normalised)
 *   - a PCA variance vector.
 *
 * It also contains a list of triangles to built a mesh as well as a mapping
 * from landmark points to the corresponding vertex-id in the mesh.
 * It is able to return instances of the model as meshes.
 */

	/**
	 * Return the value of the mean at a given vertex id.
	 *
	 * @param[in] vertex_index A vertex id.
	 * @return A homogeneous vector containing the values at the given vertex id.
	 */
    cv::Vec4f PcaModel::get_mean_at_point(int vertex_index) const
	{
		vertex_index *= 3;
		if (vertex_index >= mean.rows) {
			throw std::out_of_range("The given vertex id is larger than the dimension of the mean.");
		}
		return cv::Vec4f(mean.at<float>(vertex_index), mean.at<float>(vertex_index + 1), mean.at<float>(vertex_index + 2), 1.0f);
	};

	/**
	 * Draws a random sample from the model, where the coefficients are drawn
	 * from a standard normal (or with the given standard deviation).
	 *
	 * @param[in] sigma The standard deviation.
	 * @return A random sample from the model.
	 */
    cv::Mat PcaModel::draw_sample(float sigma = 1.0f)
	{
		std::normal_distribution<float> distribution(0.0f, sigma); // this constructor takes the stddev

		std::vector<float> alphas(get_num_principal_components());

		for (auto&& a : alphas) {
			a = distribution(engine);
		}

		return draw_sample(alphas);
	};

	/**
	 * Returns a sample from the model with the given PCA coefficients.
	 * The given coefficients should follow a standard normal distribution, i.e.
	 * not be "normalised" with their eigenvalues/variances.
	 *
	 * @param[in] coefficients The PCA coefficients used to generate the sample.
	 * @return A model instance with given coefficients.
	 */
    cv::Mat PcaModel::draw_sample(std::vector<float> coefficients)
	{
		// Fill the rest with zeros if not all coefficients are given:
		if (coefficients.size() < get_num_principal_components()) {
			coefficients.resize(get_num_principal_components());
		}
		cv::Mat alphas(coefficients);

		cv::Mat model_sample = mean + normalised_pca_basis * alphas;

		return model_sample;
	};

	} /* namespace morphablemodel */
} /* namespace eos */
