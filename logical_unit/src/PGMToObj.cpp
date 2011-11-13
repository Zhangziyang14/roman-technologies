#include "ros/ros.h"
#include "Util.h"
#include "fstream"

#define RESOLUTION 0.05 // meters / pixel
#define ORIGIN_X 0.0 // meters of leftmost pixel
#define ORIGIN_Y 0.0 // meters of downmost pixel

void addVertex(std::ofstream *file, int x, int y, double mx, double my)
{
	double rx = ORIGIN_X + x * RESOLUTION;
	double ry = ORIGIN_Y + y * RESOLUTION;
	*file << "v " << rx + mx << " 0.00 " << ry + my << std::endl;
}

void addFace(std::ofstream *file, int v1, int v2, int v3)
{
	*file << "f " << v1 << " " << v2 << " " << v3 << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PGMToObj");
	std::ofstream objFile("out.obj");

	IplImage *image = cvLoadImage("map.pgm", -1);
	if (image == NULL)
	{
		std::cout << "It failed.." << std::endl;
		return -1;
	}

	int *indices= new int[image->width * image->height];
	memset(indices, -1, image->width * image->height * sizeof(int));

	std::vector<uint32> faces;

	uint32 index = 0;
	uint32 vertIndex = 1;
	for (int y = 0; y < image->height; y++)
	{
		for (int x = 0; x < image->width; x++)
		{
			if (*getPixel<uint8>(x, y, image) >= 254)
			{
				indices[y * image->width + x] = index;
				index++;

				int vertices[4] = { -1, -1, -1, -1 }; // right-bottom, left-bottom, right-top, left-top
				if (x > 0 && y > 0 && indices[(y-1)*image->width + (x-1)] != -1) // is there a pixel at (x-1, y-1) ?
				{
					vertices[3] = faces[indices[(y-1)*image->width + (x-1)] * 3 * 2 + 4]; // copy right-bottom to left-top
				}
				if (y > 0 && indices[(y-1)*image->width + x] != -1) // is there a pixel at (x, y-1) ?
				{
					vertices[3] = faces[indices[(y-1)*image->width + x] * 3 * 2 + 2]; // copy left-bottom to left-top
					vertices[2] = faces[indices[(y-1)*image->width + x] * 3 * 2 + 4]; // copy right-bottom to right-top
				}
				if (x > 0 && indices[y * image->width + (x-1)] != -1)
				{
					vertices[3] = faces[indices[y * image->width + (x-1)] * 3 * 2 + 1]; // copy right-top to left-top
					vertices[1] = faces[indices[y * image->width + (x-1)] * 3 * 2 + 4];	// copy right-bottom to left-bottom
				}

				for (int i = 0; i < 4; i++)
				{
					// do we need to create a new vertex?
					if (vertices[i] == -1)
					{
						switch (i)
						{
						case 0: addVertex(&objFile, x, y, RESOLUTION, RESOLUTION); break;
						case 1: addVertex(&objFile, x, y, 0.0, RESOLUTION); break;
						case 2: addVertex(&objFile, x, y, RESOLUTION, 0.0); break;
						case 3: addVertex(&objFile, x, y, 0.0, 0.0); break;
						default:
							break;
						}
						vertices[i] = vertIndex;
						vertIndex++;
					}
				}

				faces.push_back(vertices[1]); // left-bottom
				faces.push_back(vertices[2]); // right-top
				faces.push_back(vertices[3]); // left-top

				faces.push_back(vertices[1]); // left-bottom
				faces.push_back(vertices[0]); // right-bottom
				faces.push_back(vertices[2]); // right-top
			}

		}
	}

	for (size_t i = 0; i < faces.size(); i = i + 3)
	{
		addFace(&objFile, faces[i], faces[i+1], faces[i+2]);
	}
	objFile.close();

	delete[] indices;

	cv::startWindowThread();
	cvNamedWindow(WINDOW_NAME);
	/*IplImage *resizedImage = cvCreateImage(cvSize(800, 600), image->depth, image->nChannels);
	cvResize(image, resizedImage);
	cvShowImage(WINDOW_NAME, resizedImage);
	cvReleaseImage(&resizedImage);
	ros::spin();*/

	return 0;
}
