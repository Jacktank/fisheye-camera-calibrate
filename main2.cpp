#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


int StereoCalib(int number_image, int nx, int ny, float SquareSize, bool has_saved_image)
{

	cv::VideoCapture left_cap(0), right_cap(1);
	if (!left_cap.isOpened())  // check if we succeeded
	{
		DBG(cout << "error:fail to load camera 1" << endl;);
		return -1;
	}
	if (!right_cap.isOpened())  // check if we succeeded
	{
		DBG(cout << "error:fail to load camera 2" << endl;);
		return -1;
	}

	int width_img = (int)left_cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int height_img = (int)left_cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	left_cap.set(CV_CAP_PROP_SATURATION, 0);
	right_cap.set(CV_CAP_PROP_SATURATION, 0);

	cv::Mat origin_image1, origin_image2;
	Mat union_img(height_img, width_img * 2, CV_8UC3);
	Rect r1(0, 0, width_img, height_img), r2(width_img, 0, width_img, height_img);
	Mat roi1 = union_img(r1);
	Mat roi2 = union_img(r2);
	
	int count_saveImages = 0;
	char dest_left[255], dest_right[255];
	int real_quit = 0;

	DBG(cout << "Press 'g' to grab two images from left camera and right camera......" << endl;);

	while (!has_saved_image)
	{
		//»ñÈ¡Ïà»úÔ­Ê¼Í¼Ïñ;

		left_cap >> origin_image1;
		right_cap >> origin_image2;
		cv::flip(origin_image1, origin_image1, 1);
		cv::flip(origin_image2, origin_image2, 1);

		//ÁªºÏ²¢ÏÔÊ¾×óÓÒÏà»úÔ­Ê¼Í¼Ïñ;
		origin_image1.convertTo(roi1, roi1.type());
		origin_image2.convertTo(roi2, roi2.type());
		cv::imshow("union_img", union_img);
		char key = cv::waitKey(10);
		if (key == 'g')
		{
			sprintf_s(dest_left, "image_save_after\\origin_left%d.jpg", count_saveImages);
			sprintf_s(dest_right, "image_save_after\\origin_right%d.jpg", count_saveImages);

			cv::imwrite(dest_left, origin_image1);
			cv::imwrite(dest_right, origin_image2);
			count_saveImages++;

			DBG(
				cout << "Succeed to grab image from camera1 and camera2: count_saveImages" << endl;
			cout << "There are still " << number_image - count_saveImages << " pictures to be processed" << endl;
			);
		}
		else if (key == 'q')
		{
			real_quit++;

			if (number_image - count_saveImages <= 0)
			{
				DBG(
					cout << "½ØÈ¡Í¼ÏñÖ¡¹ý³ÌÍê³É..." << endl;
				cout << "¹²³É¹¦½ØÈ¡---" << count_saveImages << "Ö¡Í¼Ïñ£¡£¡£¡" << endl;
				);
				break;
			}
			else
			{

				if (real_quit == 2)
				{
					DBG(
						cout << "Are you sure to quit calibrating Stereo Cameras?" << endl;
					cout << "re-input key 'q' to quit" << endl;
					);
				}
				if (real_quit == 3)
				{
					DBG(
						cout << "Warning: You have quit calibrating!!! Goodby!" << endl;
					);
					return -1;
				}

				DBG(
					cout << "There are still " << number_image - count_saveImages << " pictures to be processed!!!" << endl;
				cout << "Please to continues!" << endl;
				);
				
			}
		}
	}

	int total_per_image = nx*ny;
	Size sz(nx, ny);

	vector<vector<Point2f>> imagePoints[2];
	vector<vector<Point3f>> objectPoints(1);	
	vector<Point2f> corners_buf_left, corners_buf_right;

	Mat left_gray, right_gray;

	Mat M1;
	Mat D1;
	Mat M2;
	Mat D2;
	vector<Mat> R1, R2, T1, T2;
	Mat R, T, E, F, P1, P2, R_l, R_r, H1, H2, Q;
	Mat mx1, mx2, my1, my2;

	int counter = 0;
	int successes = 0;

	while (counter < number_image)
	{
		sprintf_s(dest_left, "image_save_after\\origin_left%d.jpg", counter);
		sprintf_s(dest_right, "image_save_after\\origin_right%d.jpg", counter);
		origin_image1 = cv::imread(dest_left, 1);
		origin_image2 = cv::imread(dest_right, 1);

		if (origin_image1.empty())
		{
			DBG(cout << "error: fail to load the image1:" << counter << "!!!" << endl;);
			counter++;
			continue;
			/*return -1;*/
		}
		if (origin_image2.empty())
		{
			DBG(cout << "error: fail to load the image2:" << counter << "!!!" << endl;);
			counter++;
			continue;
			/*return -1;*/
		}
		counter++;

		cvtColor(origin_image1, left_gray, cv::COLOR_RGB2GRAY);
		cvtColor(origin_image2, right_gray, cv::COLOR_RGB2GRAY);

		int patternfound_1 = findChessboardCorners(left_gray, sz, corners_buf_left, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);
		int patternfound_2 = findChessboardCorners(right_gray, sz, corners_buf_right, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);
		if (!(patternfound_1&patternfound_2))
		{
			DBG(cout << "µÚ" << counter - 1 << "Ö¡Í¼Æ¬ÎÞ·¨ÕÒµ½ÆåÅÌ¸ñËùÓÐ½Çµã!" << endl;
			cout << corners_buf_left.size() << ":" << corners_buf_right.size() << endl;);

			//cv::imshow("RePlay",origin_image1);
			//cv::waitKey(0);
			continue;
		}
		else
		{
			DBG(cout << "µÚ" << counter - 1 << "Ö¡Í¼Ïñ³É¹¦»ñµÃ" << corners_buf_left.size() << "¸ö½Çµã..." << endl;);

			cornerSubPix(left_gray, corners_buf_left, Size(7, 7), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.001));
			cornerSubPix(right_gray, corners_buf_right, Size(7, 7), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.001));

			drawChessboardCorners(origin_image1, sz, Mat(corners_buf_left), patternfound_1);
			drawChessboardCorners(origin_image2, sz, Mat(corners_buf_right), patternfound_2);
			cout << "ÔÚÔ´Í¼ÏñÉÏ»æÖÆ½Çµã¹ý³ÌÍê³É...\n\n";

			origin_image1.convertTo(roi1, roi1.type());
			origin_image2.convertTo(roi2, roi2.type());
			cv::imshow("union_img", union_img);
			cv::waitKey(30);
		}

		//ÄÜ¼ì²â³öµ±Ç°×óÓÒÑù±¾Í¼ÏñËùÓÐ½Çµã£¬Ôò´æÈëimage_pointsÖÐ;
		if (total_per_image == corners_buf_left.size() & total_per_image == corners_buf_right.size())
		{
			imagePoints[0].push_back(corners_buf_left);
			imagePoints[1].push_back(corners_buf_right);
			successes++;
		}
		DBG(cout << "Please input a char to continue!" << endl;);
		cv::waitKey(0);
	}
	//³õÊ¼»¯objectPoints;
	InitCorners3D(objectPoints, sz, successes, SquareSize);//³É¹¦¼ì²â¼¸ÕÅÍ¼ÏñµÄ½Çµã£¬¾Í±ê¶¨¼¸ÕÅ;

	cv::calibrateCamera(objectPoints, imagePoints[0], cv::Size(width_img, height_img), M1, D1, R1, T1);//CV_CALIB_RATIONAL_MODEL);//,CV_CALIB_FIX_K3);//InputArrayOfArray;
	cv::calibrateCamera(objectPoints, imagePoints[1], cv::Size(width_img, height_img), M2, D2, R2, T2);//,CV_CALIB_RATIONAL_MODEL );//,CV_CALIB_FIX_K3);//InputArrayOfArray;

	DBG(cout << "Running stereo calibration ..." << endl;);


}


int main()
{

		cv::FileStorage fs("omni_calib_data.xml", cv::FileStorage::READ);
		std::vector<cv::Mat> objectPoints, imagePoints;
		cv::Size imgSize;
		fs["objectPoints"] >> objectPoints;
		fs["imagePoints"] >> imagePoints;
		fs["imageSize"] >> imgSize;


		cv::Mat K, xi, D, idx;

		int flags = 0;

		cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);

		std::vector<cv::Mat> rvecs, tvecs;

		double rms = cv::omnidir::calibrate(objectPoints, imagePoints, imgSize, K, xi, D, rvecs, tvecs, flags, critia, idx);

		return 0;
}








int main()
{

	int image_count = 19;
  cv::Size board_size(6, 9);
	cv::Size2f square_size(25.2, 25.2);
	int	fullcornersNum = board_size.area();
	string path_ChessboardImage = "origin_left";
	
	int count = 0;
	int successImageNum = 0;
	vector<Mat> image_Seq;
	vector<vector<Point2f>> corners_Seq;

	for (int i = 0; i != image_count; i++)
	{
		cout << "Frame #" << i + 1 << "..." << endl;
		string image_Name;
		stringstream stream;
		stream << i;
		stream >> image_Name;
	  image_Name = path_ChessboardImage + image_Name + ".jpg";
		cv::Mat image = imread(image_Name);
		Mat image_gray;
		cvtColor(image, image_gray, CV_RGB2GRAY);
		vector<Point2f> corners;
		bool patternFound = findChessboardCorners(image_gray, board_size, corners,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (!patternFound || fullcornersNum != corners.size())
		{
			cout << "can not find chessboard corners!\n";
			continue;
		}
		else
		{
			cornerSubPix(image_gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			count = count + corners.size();
			corners_Seq.push_back(corners);
			successImageNum = successImageNum + 1;
			image_Seq.push_back(image);
		}
	}
	/************************************************************************
*************************************************************************/
	vector<vector<Point3f>>  object_Points;        

	Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0)); 
	vector<int>  point_counts;
	
	float width = (board_size.width - 1) / 2.0*square_size.width;
	float height = (board_size.height - 1) / 2.0*square_size.height;
	
	for (int t = 0; t<successImageNum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i<board_size.height; i++)
		{
			for (int j = 0; j<board_size.width; j++)
			{
				Point3f tempPoint;
				tempPoint.x = j*square_size.width- width;//???????????
				tempPoint.y = i*square_size.height- height;//???????????
				tempPoint.z = 0;
				tempPointSet.push_back(tempPoint);
			}
		}
		object_Points.push_back(tempPointSet);
	}
	for (int i = 0; i< successImageNum; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
	
	Size image_size = image_Seq[0].size();
	cv::Matx33d intrinsic_matrix;    
	cv::Vec4d distortion_coeffs;     
	std::vector<cv::Vec3d> rotation_vectors;                          
	std::vector<cv::Vec3d> translation_vectors;                       
	int flags = 0;
	flags |= cv::omnidir::CALIB_RECOMPUTE_EXTRINSIC;
	flags |= cv::omnidir::CALIB_CHECK_COND;
	flags |= cv::omnidir::CALIB_FIX_SKEW;

	cv::Mat K, xi, D, idx;
	int flags = 0;
	cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);

	std::vector<cv::Mat> rvecs, tvecs;

	double rms = cv::omnidir::calibrate(objectPoints, imagePoints, imgSize, K, xi, D, rvecs, tvecs, flags, critia, idx);

	return 0;

}
