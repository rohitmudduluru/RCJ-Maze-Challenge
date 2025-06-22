// opencv stuff
#include <unistd.h> // serial comms
#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

// file writing
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int ct = 0;
using namespace std;
using namespace cv;

//Some file writing thing
#define WRITENEW 0
FileStorage label_file, data_file;
Mat label_data, data_data;
int total_save = 0;

//compare contour area function
bool contour_compare(const vector<Point> &a, const vector<Point> &b) {
return contourArea(a) < contourArea(b);
}

void save_letter_data(char label, Mat data){

	// resize the letter
	resize(data, data, Size(20,20));
	imshow(to_string(label), data);

	// flatten
	cout << "data size = " << data.size() << endl;
	Mat flat = data.reshape(1,data.total());
	cout << "data size = " << flat.size() << endl;

	// write it out to a file...
	if(WRITENEW){
		data_data.push_back(flat);
		total_save++;
		label_data.push_back(float(label));
		printf("\n\nWROTE DATA\n\n");
	}
}

void proc_letter(Mat og_img, Mat img){
	cout << "proc_letter start" << endl;
	cvtColor(img, img, COLOR_BGR2GRAY);
	GaussianBlur(img, img, Size(9,9), 0, 0);
	threshold(img, img, 80, 255, THRESH_BINARY_INV+THRESH_OTSU);
	imshow("thresh", img);
	
	vector<vector<Point>> contours;
	findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	sort(contours.begin(), contours.end(), contour_compare);
	
	Mat letter;
	Rect br;
	Mat blank = Mat::zeros(img.size(), CV_8UC1);
	cout << "At contours" << endl;
	cout << contours.size() << endl;
	for(auto& cnt : contours){
		cout << "in countours" << endl;
		double a = contourArea(cnt);
		// filter with area
		if(!(a < 2000 && a > 300)){
			drawContours(og_img, vector<vector<Point>>(1,cnt), -1, Scalar(0, 0, 255), 1);
			cout << "filtered out cuz of area thingy" << endl;
			continue;
		}
		
		//make a bounding rect
		br = boundingRect(cnt);
		float ar = (float)br.height/br.width;
		
		printf("ar: %.2f\n", ar);

		if(ar > 1.5 || ar < 0.5){
		drawContours(og_img, vector<vector<Point>>(1,cnt), -1, Scalar(0, 255, 255), 1);
		continue;
		}
		
		// should be a letter by this point
		rectangle(og_img, br, Scalar(0, 255, 0), 1);
		imshow("ogyk", og_img);
		
		cout << "passes making rectangle" << endl;
		// redraw contour on a blank image
		drawContours(blank, vector<vector<Point>>(1, cnt), -1, 255, -1);
		imshow("blank", blank);
		
		cout << "i got to the slicing" << endl;
		cout << br.x << " " << br.y << " " << br.width << " " << br.height << endl;
		// slice the letter so the rect is a little bigger
		if(br.x < 10 || br.y < 10 || br.x + br.width + 10 > 319 || br.height + br.y + 10 > 239){
			continue;
		}
		letter = blank(Rect(br.x - 10, br.y - 10, br.width + 20, br.height + 20));
		imshow("letter", letter);
		cout << "i passed the slicing" << endl;


		RotatedRect rr = minAreaRect(cnt);
		printf("ANGLE: %.2f\n", rr.angle);

		cout << "center: " << rr.center << endl;

		// warp affine stuff - make it some 90 degree angle
		Mat rot_matrix = getRotationMatrix2D(Point2f(letter.cols/2.0F, letter.rows/2.0F), rr.angle, 1.0);
		Mat rotated_letter(Size(letter.size().height, letter.size().width), letter.type());
		warpAffine(letter, rotated_letter, rot_matrix, letter.size());
		imshow("rotated letter", rotated_letter);

		//start defining the letter
		printf("\nWhich letter is it???\n");
		char key = 255;
		Mat save_letter;
		save_letter = rotated_letter.clone();
		while(key != 'h' && key != 's' && key != 'u' && key != 'q'){

			printf("save or rotate??\n");
			imshow("Save image", save_letter);
			key = (char)cv::waitKey(0);

			if(key == 'h' || key == 's' || key == 'u'){
				// get rid of border around letter
				vector<vector<Point>> letter_contours;
				findContours(save_letter, letter_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				sort(letter_contours.begin(), letter_contours.end(), contour_compare);
				//vector<Point> final_letter = letter_contours[0];
				Rect final_letter = boundingRect(letter_contours[0]);
				Mat res = save_letter(final_letter);

				// save the final sliced and cropped letter data to file
				save_letter_data(key, res);
				ct++;
			}
			else if(key == 'q'){
				printf("EXIT\n");
			}
			else{
				printf("ROTATING\n");
			}

			// rotate the image
			rot_matrix = getRotationMatrix2D(Point2f(save_letter.cols/2.0F, save_letter.rows/2.0F), 90, 1.0);
			warpAffine(save_letter, save_letter, rot_matrix, save_letter.size());


		}//while end

	}//for end



	cv::imshow("Basic image", og_img);
}

int main(){
	//start cam
	Mat img;
	lccv::PiCamera cam;
	
	cam.options->camera = 0;
	cam.options->video_width=320;//640;
	cam.options->video_height=240;//480;
	cam.options->framerate=30;
	cam.options->verbose=true;
	cam.startVideo();
	namedWindow("hello",cv::WINDOW_NORMAL);
	char key = 255;
	
	// open the file for writing
	if(WRITENEW){
		label_file = FileStorage("labels.txt", FileStorage::APPEND);
		data_file = FileStorage("data.txt", FileStorage::APPEND);
	}
	
	while(true){
		//check that the cam works
		if(!cam.getVideoFrame(img,99999999)){
			std::cout << "Camera error" << std::endl;
			break;
		}
		
		Mat og_img = img.clone();
		cout << "START DONT PRESS Q CUZ U HAVE MORE DATA: " << ct << endl;
		key = (char)cv::waitKey(0);
		if(key == 'q'){
			cout << "BYE" << endl;
			break;
		}
		else if(key == 's'){
			proc_letter(og_img, img);
		}
	}
	if(WRITENEW){
	label_file << "LABELS" << label_data;
	Mat m = data_data.reshape(1, total_save);
	data_file << "DATA" << m;
	label_file.release();
	data_file.release();
	printf("\n\nDONE DONE DONE DONE\n\n");
	}

	cam.stopVideo();
	destroyAllWindows();
}


