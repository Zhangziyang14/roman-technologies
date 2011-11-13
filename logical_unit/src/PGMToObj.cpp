#include "ros/ros.h"
#include "Util.h"
#include "fstream"

#define RESOLUTION 0.05 // meters / pixel
#define ORIGIN_X 0.0 // meters of leftmost pixel
#define ORIGIN_Y 0.0 // meters of downmost pixel

void addVertices(std::ofstream *file, cv::Point2d p[])
{
	for (int i = 0; i < 4; i++)
		*file << "v " << p[i].x << " 0 " << p[i].y << std::endl;
}

void addFace(std::ofstream *file, int vStart)
{
	*file << "f " << vStart << " " << vStart + 1 << " " << vStart + 2 << " " << vStart + 3 << std::endl;
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

	std::vector<uint32> faces;

	uint32 vertIndex = 1;
	cv::Point2d vertices[4]; // left-top, left-bottom, right-bottom, right-top
	bool buildingPolygon = false;
	for (int y = 0; y < image->height; y++)
	{
		for (int x = 0; x < image->width; x++)
		{
			if (*getPixel<uint8>(x, y, image) >= 254)
			{
				if (buildingPolygon == false)
				{
					// if we are starting a line, fill the leftmost vertices
					vertices[0].x = ORIGIN_X + x * RESOLUTION;
					vertices[0].y = ORIGIN_Y + y * RESOLUTION;
					vertices[1].x = ORIGIN_X + x * RESOLUTION;
					vertices[1].y = ORIGIN_Y + y * RESOLUTION + RESOLUTION;
				}

				// fill the rightmost vertices regardless of extending or starting a new line
				vertices[2].x = ORIGIN_X + x * RESOLUTION + RESOLUTION;
				vertices[2].y = ORIGIN_Y + y * RESOLUTION + RESOLUTION;
				vertices[3].x = ORIGIN_X + x * RESOLUTION + RESOLUTION;
				vertices[3].y = ORIGIN_Y + y * RESOLUTION;

				// we are now building a line polygon
				buildingPolygon = true;
			}
			else // obstacle or void
			{
				// end of polygon, add it to the file
				if (buildingPolygon)
				{
					addVertices(&objFile, vertices);
					for (int i = 0; i < 4; i++)
						faces.push_back(vertIndex + i);

					vertIndex += 4;
					buildingPolygon = false;
				}
			}
		}

		// end of line while building polygon? add it to obj file
		if (buildingPolygon)
		{
			addVertices(&objFile, vertices);
			for (int i = 0; i < 4; i++)
				faces.push_back(vertIndex + i);

			vertIndex += 4;
			buildingPolygon = false;
		}
	}

	for (size_t i = 0; i < faces.size(); i = i + 4)
		addFace(&objFile, faces[i]);

	objFile.close();

	cv::startWindowThread();
	cvNamedWindow(WINDOW_NAME);
	return 0;
}
