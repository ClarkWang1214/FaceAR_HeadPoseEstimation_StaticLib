///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __FACEAR_PARAM_H
#define __FACEAR_PARAM_H

using namespace cv;
using namespace std;

namespace FaceARTracker
{

struct FaceARParameters
{
	int num_optimisation_iteration;
	bool limit_pose;
	bool validate_detections;
	double validation_boundary;
	vector<int> window_sizes_small;
	vector<int> window_sizes_init;
	vector<int> window_sizes_current;
	double face_template_scale;	
	bool use_face_template;
	string model_location;
	double sigma;
    double reg_factor;
    double weight_factor;
	bool multi_view;
	int reinit_video_every;
	enum FaceDetector{HAAR_DETECTOR, HOG_SVM_DETECTOR};

	string face_detector_location;
	FaceDetector curr_face_detector;
	bool quiet_mode;
	bool refine_hierarchical;
	bool refine_parameters;
	bool track_gaze;

    FaceARParameters()
	{
		init();
	}

    FaceARParameters(vector<string> &arguments)
	{
	    init(); 


		boost::filesystem::path root = boost::filesystem::path(arguments[0]).parent_path();

		bool* valid = new bool[arguments.size()];
		valid[0] = true;

		for(size_t i = 1; i < arguments.size(); ++i)
		{
			valid[i] = true;

			if (arguments[i].compare("-mloc") == 0) 
			{                    
				string model_loc = arguments[i + 1];
				model_location = model_loc;
				valid[i] = false;
				valid[i+1] = false;
				i++;

			}
			if (arguments[i].compare("-clm_sigma") == 0) 
			{                    
				stringstream data(arguments[i + 1]);
				data >> sigma;
				valid[i] = false;
				valid[i+1] = false;
				i++;
			}
			else if(arguments[i].compare("-w_reg") == 0)
			{
				stringstream data(arguments[i + 1]);
				data >> weight_factor;
				valid[i] = false;
				valid[i+1] = false;
				i++;
			}
			else if(arguments[i].compare("-reg") == 0)
			{
				stringstream data(arguments[i + 1]);
				data >> reg_factor;
				valid[i] = false;
				valid[i+1] = false;
				i++;
			}
			else if (arguments[i].compare("-multi_view") == 0) 
			{                    

				stringstream data(arguments[i + 1]);
				int m_view;
				data >> m_view;

				multi_view = (bool)(m_view != 0);
				valid[i] = false;
				valid[i+1] = false;
				i++;
			}
			else if(arguments[i].compare("-validate_detections") == 0)
			{
				stringstream data(arguments[i + 1]);
				int v_det;
				data >> v_det;

				validate_detections = (bool)(v_det != 0);
				valid[i] = false;
				valid[i+1] = false;
				i++;
			}
			else if(arguments[i].compare("-n_iter") == 0)
			{
				stringstream data(arguments[i + 1]);											
				data >> num_optimisation_iteration;

				valid[i] = false;
				valid[i+1] = false;
				i++;
			}
			else if (arguments[i].compare("-q") == 0) 
			{                    

				quiet_mode = true;

				valid[i] = false;
			}
			else if (arguments[i].compare("-clmwild") == 0) 
			{                    
				window_sizes_init = vector<int>(4);
				window_sizes_init[0] = 15; window_sizes_init[1] = 13; window_sizes_init[2] = 11; window_sizes_init[3] = 9;

				sigma = 1.25;
				reg_factor = 35;
				weight_factor = 2.5;
				num_optimisation_iteration = 10;

				valid[i] = false;

				curr_face_detector = HOG_SVM_DETECTOR;

			}
			else if (arguments[i].compare("-help") == 0)
			{
                cout << "FaceAR parameters are defined as follows: "
                        "-mloc <location of model file> "
                        "-pdm_loc <override pdm location> "
                        "-w_reg <weight term for patch rel.> "
                        "-reg <prior regularisation> "
                        "-clm_sigma <float sigma term> "
                        "-fcheck <should face checking be done 0/1> "
                        "-n_iter <num EM iterations> "
                        "-clwild (for in the wild images) "
                        "-q (quiet mode)" << endl; // Inform the user of how to use the program
			}
		}

		for(int i=arguments.size()-1; i >= 0; --i)
		{
			if(!valid[i])
			{
				arguments.erase(arguments.begin()+i);
			}
		}

		if(!boost::filesystem::exists(boost::filesystem::path(model_location)))
		{
			model_location = (root / model_location).string();
			if(!boost::filesystem::exists(boost::filesystem::path(model_location)))
			{
				std::cout << "Could not find the landmark detection model to load" << std::endl;
			}
		}

	}

	private:
		void init()
		{
			num_optimisation_iteration = 5;
			validate_detections = true;
			refine_hierarchical = true;
			refine_parameters = true;

			window_sizes_small = vector<int>(4);
			window_sizes_init = vector<int>(4);

			window_sizes_small[0] = 0;
			window_sizes_small[1] = 9;
			window_sizes_small[2] = 7;
			window_sizes_small[3] = 5;

			window_sizes_init.at(0) = 11;
			window_sizes_init.at(1) = 9;
			window_sizes_init.at(2) = 7;
			window_sizes_init.at(3) = 5;
			
			face_template_scale = 0.3;
			use_face_template = false;

			window_sizes_current = window_sizes_init;
            /// TODO:Keegan.Ren
            model_location = "../model/model/main_ccnf_general.txt";

			sigma = 1.5;
			reg_factor = 25;
            weight_factor = 0;

			validation_boundary = -0.45;

			limit_pose = true;
			multi_view = false;

			reinit_video_every = 4;
						
            /// TODO:Keegan.Ren
			// Face detection
			#if OS_UNIX
                face_detector_location = "../model/classifiers/haarcascade_frontalface_alt.xml";
			#else
                face_detector_location = "../model/classifiers/haarcascade_frontalface_alt.xml";
			#endif

			quiet_mode = false;

			// By default use HOG SVM
            /// TODO : HOG_SVM_DETECTOR  or  HAAR_DETECTOR
			curr_face_detector = HOG_SVM_DETECTOR;

			track_gaze = false;
		}
};

}

#endif // __CLM_PARAM_H

