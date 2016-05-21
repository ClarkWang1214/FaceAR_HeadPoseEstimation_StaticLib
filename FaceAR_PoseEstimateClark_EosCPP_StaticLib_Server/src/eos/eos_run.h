#ifndef EOS_RUN_H
#define EOS_RUN_H

#include "eos/core/Landmark.h"
#include "eos/core/LandmarkMapper.h"
#include "eos/fitting/nonlinear_camera_estimation.h"
#include "eos/fitting/linear_shape_fitting.h"
#include "eos/render/utils.h"
#include "eos/render/texture_extraction.h"

#include "FaceAR_core.h"

using namespace eos;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using eos::core::Landmark;
using eos::core::LandmarkCollection;
using cv::Mat;
using cv::Vec2f;
using cv::Vec3f;
using cv::Vec4f;
using std::cout;
using std::endl;
using std::vector;
using std::string;

bool getModel(fs::path &modelfile, fs::path &mappingsfile, fs::path &outputfile, fs::path &isomapfile,
              FaceARTracker::FaceAR &facear_model, cv::Mat &captured_image_eos, int &frame_count,
              render::Mesh &mesh, std::string &obj_root);
//{
//    /// TODO : eos
//    std::cout << "facear_model.pdm.NumberOfPoints() = " << facear_model.pdm.NumberOfPoints() << endl;
//    // face 68 pointers
//    /// every face
//    morphablemodel::MorphableModel morphable_model;
//    try {
//        morphable_model = morphablemodel::load_model(modelfile.string());
//    }
//    catch (const std::runtime_error& e) {
//        cout << "Error loading the Morphable Model: " << e.what() << endl;
//        return EXIT_FAILURE;
//    }
//    core::LandmarkMapper landmark_mapper = mappingsfile.empty() ? core::LandmarkMapper() : core::LandmarkMapper(mappingsfile);

//    LandmarkCollection<Vec2f> landmarks;
//    landmarks.reserve(68);
//    for (size_t i = 0; i < facear_model.pdm.NumberOfPoints(); ++i)
//    {
//        cv::Point pt_save = cv::Point(facear_model.detected_landmarks.at<double>(i),
//                                      facear_model.detected_landmarks.at<double>(i+facear_model.pdm.NumberOfPoints()));
//        Landmark<Vec2f> landmark;
//        /// input
//        landmark.name = std::to_string(i+1);
//        landmark.coordinates[0] = pt_save.x;
//        landmark.coordinates[1] = pt_save.y;
//        //cout << shapes[i].part(j) << "\t";
//        landmark.coordinates[0] -= 1.0f;
//        landmark.coordinates[1] -= 1.0f;
//        landmarks.emplace_back(landmark);
//    }

//    // These will be the final 2D and 3D points used for the fitting:
//    vector<Vec4f> model_points; // the points in the 3D shape model
//    vector<int> vertex_indices; // their vertex indices
//    vector<Vec2f> image_points; // the corresponding 2D landmark points

//    // Sub-select all the landmarks which we have a mapping for (i.e. that are defined in the 3DMM):
//    for (int i = 0; i < landmarks.size(); ++i) {
//        auto converted_name = landmark_mapper.convert(landmarks[i].name);
//        if (!converted_name) { // no mapping defined for the current landmark
//            continue;
//        }
//        int vertex_idx = std::stoi(converted_name.get());
//        Vec4f vertex = morphable_model.get_shape_model().get_mean_at_point(vertex_idx);
//        model_points.emplace_back(vertex);
//        vertex_indices.emplace_back(vertex_idx);
//        image_points.emplace_back(landmarks[i].coordinates);
//    }

//    // Estimate the camera (pose) from the 2D - 3D point correspondences
//    fitting::OrthographicRenderingParameters rendering_params = fitting::estimate_orthographic_camera(image_points, model_points, captured_image_eos.cols, captured_image_eos.rows);
//    Mat affine_from_ortho = fitting::get_3x4_affine_camera_matrix(rendering_params, captured_image_eos.cols, captured_image_eos.rows);

//    // The 3D head pose can be recovered as follows:
//    float yaw_angle = glm::degrees(rendering_params.r_y);
//    // and similarly for pitch (r_x) and roll (r_z).

//    // Estimate the shape coefficients by fitting the shape to the landmarks:
//    vector<float> fitted_coeffs = fitting::fit_shape_to_landmarks_linear(morphable_model, affine_from_ortho, image_points, vertex_indices);

//    // Obtain the full mesh with the estimated coefficients:
//    /*render::Mesh mesh*/
//    mesh = morphable_model.draw_sample(fitted_coeffs, vector<float>());

//    // Extract the texture from the image using given mesh and camera parameters:
//    Mat isomap = render::extract_texture(mesh, affine_from_ortho, captured_image_eos);

//    //                // Save the mesh as textured obj:
//    //                outputfile += fs::path(".obj");
//    //                render::write_textured_obj(mesh, outputfile.string());

//    //                // And save the isomap:
//    //                outputfile.replace_extension(".isomap.png");
//    //                cv::imwrite(outputfile.string(), isomap);

//    ///// save obj
//    std::stringstream strOBJ;
//    strOBJ << /*"result/" <<*/ std::setw(10) << std::setfill('0') << frame_count/* << ".obj"*/;

//    // Save the mesh as textured obj:
//    outputfile += fs::path(".obj");
//    render::write_textured_obj(mesh, strOBJ.str().append((string)".obj")/*outputfile.string()*/);

//    // And save the isomap:
//    outputfile.replace_extension(".isomap.png");
//    cv::imwrite(/*(string)"result/" + */strOBJ.str().append((string)".isomap.png")/*outputfile.string()*/, isomap);

//    cv::imshow("isomap_png", isomap);
//    cv::waitKey(1);

//    //cout << "Finished fitting and wrote result mesh and isomap to files with basename " << outputfile.stem().stem() << "." << endl;
//    outputfile.clear();

//    return 1;
//}

#endif // EOS_RUN_H
