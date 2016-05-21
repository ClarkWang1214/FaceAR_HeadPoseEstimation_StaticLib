/*
 * Eos - A 3D Morphable Model fitting library written in modern C++11/14.
 *
 * File: include/eos/core/LandmarkMapper.hpp
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

#include <eos/core/LandmarkMapper.h>

namespace eos {
    namespace core {
    /**
     * @brief Constructs a new landmark mapper from a file containing
     * mappings from one set of landmark identifiers to another.
     *
     * In case the file contains no mappings, a landmark mapper
     * that performs an identity mapping is constructed.
     *
     * @param[in] filename A file with landmark mappings.
     * @throws runtime_error if there is an error loading
     *         the mappings from the file.
     */
    LandmarkMapper::LandmarkMapper(boost::filesystem::path filename)
    {
        using std::string;
        using boost::property_tree::ptree;
        ptree configtree;
        try {
            boost::property_tree::info_parser::read_info(filename.string(), configtree);
        }
        catch (const boost::property_tree::ptree_error& error) {
            throw std::runtime_error(string("LandmarkMapper: Error reading landmark-mappings file: ") + error.what());
        }

        try {
            ptree pt_landmark_mappings = configtree.get_child("landmarkMappings");
            for (auto&& mapping : pt_landmark_mappings) {
                landmark_mappings.insert(make_pair(mapping.first, mapping.second.get_value<string>()));
            }
        }
        catch (const boost::property_tree::ptree_error& error) {
            throw std::runtime_error(string("LandmarkMapper: Error while parsing the mappings file: ") + error.what());
        }
        catch (const std::runtime_error& error) {
            throw std::runtime_error(string("LandmarkMapper: Error while parsing the mappings file: ") + error.what());
        }
    }

    /**
     * @brief Converts the given landmark name to the mapped name.
     *
     * @param[in] landmark_name A landmark name to convert.
     * @return The mapped landmark name if a mapping exists, an empty optional otherwise.
     * @throws out_of_range exception if there is no mapping
     *         for the given landmarkName.
     */
    boost::optional<std::string> LandmarkMapper::convert(std::string landmark_name) const
    {
        if (landmark_mappings.empty()) {
            // perform identity mapping, i.e. return the input
            return landmark_name;
        }
        else {
            auto&& converted_landmark = landmark_mappings.find(landmark_name);
            if (converted_landmark != std::end(landmark_mappings)) {
                // landmark mapping found, return it
                return converted_landmark->second;
            }
            else { // landmark_name does not match the key of any element in the map
                return boost::none;
            }
        }
    }

    /**
     * @brief Returns the number of loaded landmark mappings.
     *
     * @return The number of landmark mappings.
     */
    size_t LandmarkMapper::size() const
    {
        return landmark_mappings.size();
    }

    }
}
