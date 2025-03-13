#include <chrono>
#include "include/yolo.h"
#include "include/utils.h"

#ifdef _YOLO_OPENCV
	#include "yolo_opencv.h"
#endif // _YOLO_OpenCV

void YOLO::infer(const cv::Mat &img, bool save_result, bool show_result)
{
	// if(!std::filesystem::exists(file_path))
	// {
	// 	std::cerr << "file not exists!" << std::endl;
	// 	std::exit(-1);
	// }

	m_draw_result = save_result || show_result;
	// std::string extension = std::filesystem::path(file_path).extension().string();
	// if (extension == ".bmp" || extension == ".jpg" || extension == ".png")
	// {
		// m_image = cv::imread(file_path);
		m_image = img.clone();
		if (m_image.empty())
		{
			std::cerr << "read image empty!" << std::endl;
			std::exit(-1);
		}

		//warm up
		// for(int i=0; i<10; ++i)
		// {
			// pre_process();
			// process();
			// post_process();
		// }	

		auto start = std::chrono::steady_clock::now();
		// for(int i=0; i<1000; ++i)
		// {
		// 	pre_process();
		// 	process();
		// 	post_process();
		// }		
		pre_process();
		process();
		post_process();
		auto end = std::chrono::steady_clock::now();	
		std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);		
		std::cout << "avg cost run on 1000 times:" << duration.count() << "ms" << std::endl;

		// if (save_result)
		// {
		// 	std::string result_name = "./result/" + std::string(argv[1]) + std::string(argv[2]) + std::string(argv[3]) + std::string(argv[4]) + std::string(argv[5]) + ".jpg";
		// 	cv::imwrite(result_name, m_result);
		// }
		if (show_result)
		{
			cv::imshow("result", m_result);
			cv::waitKey(0);
			cv::destroyAllWindows();
		}
	// }
	// else if (extension == ".mp4")
	// {
	// 	cv::VideoCapture cap;
	// 	if (!cap.open(file_path))
	// 	{
	// 		std::cerr << "read video empty!" << std::endl;
	// 		std::exit(-1);
	// 	}

	// 	cv::VideoWriter wri;
	// 	if (save_result)
	// 	{
	// 		std::string result_name = "./result/" + std::string(argv[1]) + std::string(argv[2]) + std::string(argv[3]) + std::string(argv[4]) + std::string(argv[5]) + ".avi";
	// 		int width = int(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  //   		int height = int(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
	// 		wri.open(result_name, cv::VideoWriter::fourcc('M', 'P', '4', '2'), 30, cv::Size(width, height));
	// 	}

	// 	auto start = std::chrono::steady_clock::now();

	// 	while (true)
	// 	{
	// 		cap.read(m_image);
	// 		if (m_image.empty())
	// 		{
	// 			break;
	// 		}
	// 		m_result = m_image.clone();

	// 		pre_process();
	// 		process();
	// 		post_process();

	// 		if (save_result)
	// 		{
	// 			wri << m_result;
	// 		}
	// 		if (show_result)
	// 		{
	// 			cv::imshow("result", m_result);
	// 			cv::waitKey(1);
	// 		}
	// 	}	

	// 	auto end = std::chrono::steady_clock::now();	
	// 	std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);		
	// 	std::cout << duration.count() * 1000 << "ms" << std::endl;
	// 	cap.release();

	// 	if (save_result)
	// 	{
	// 		wri.release();
	// 	}
	// 	if (show_result)
	// 	{
	// 		cv::destroyAllWindows();
	// 	}
	// }
	// else
	// {
	// 	std::cerr << "supported input file types: .bmp .jpg .png .mp4" << std::endl;
	// 	std::exit(-1);
	// }
}

CreateFactory& CreateFactory::instance()
{
	static CreateFactory create_factory;
	return create_factory;
}

void CreateFactory::register_class(const Backend_Type& backend_type, const Task_Type& task_type, CreateFunction create_function)
{
	m_create_registry[backend_type][task_type] = create_function;
}

std::unique_ptr<YOLO> CreateFactory::create(const Backend_Type& backend_type, const Task_Type& task_type)
{
	if (backend_type >= m_create_registry.size())
	{
		std::cerr << "unsupported backend type!" << std::endl;
		std::exit(-1);
	}
	if (task_type >= m_create_registry[task_type].size())
	{
		std::cerr << "unsupported task type!" << std::endl;
		std::exit(-1);
	}

	std::unique_ptr<YOLO> yolo = m_create_registry[backend_type][task_type]();
	if(yolo == nullptr)
	{
		std::cerr << "algo create failed!" <<std::endl;
		std::exit(-1);
	}
	else 
	{
		return yolo;
	}
}

CreateFactory::CreateFactory()
{
	m_create_registry.resize(5, std::vector<CreateFunction>(3));

#ifdef _YOLO_OPENCV
	register_class(Backend_Type::OpenCV, Task_Type::Classify, []() -> std::unique_ptr<YOLO> { return std::make_unique<YOLO_OpenCV_Classify>(); });
	register_class(Backend_Type::OpenCV, Task_Type::Detect, []() -> std::unique_ptr<YOLO> { return std::make_unique<YOLO_OpenCV_Detect>(); });
	register_class(Backend_Type::OpenCV, Task_Type::Segment, []() -> std::unique_ptr<YOLO> { return std::make_unique<YOLO_OpenCV_Segment>(); });
#else
	register_class(Backend_Type::OpenCV, Task_Type::Classify, []() -> std::unique_ptr<YOLO> { return nullptr; });
	register_class(Backend_Type::OpenCV, Task_Type::Detect, []() -> std::unique_ptr<YOLO> { return nullptr; });
	register_class(Backend_Type::OpenCV, Task_Type::Segment, []() -> std::unique_ptr<YOLO> { return nullptr; });
#endif // _YOLO_OpenCV
}
