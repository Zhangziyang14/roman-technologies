#include "ros/ros.h"
#include "image_processing/Util.h"
#include "fstream"

#define RESOLUTION 0.05 // meters / pixel
#define ORIGIN_X 0.0 // meters of leftmost pixel
#define ORIGIN_Y 0.0 // meters of downmost pixel

void addVertices(std::ofstream *file, cv::Point2d p[], double z)
{
	for (int i = 0; i < 4; i++)
		*file << "v " << p[i].x << " " << z << " " << p[i].y << std::endl;
}

void addFace(std::ofstream *file, int v1, int v2, int v3, int v4)
{
	*file << "f " << v1 << " " << v2 << " " << v3 << " " << v4 << std::endl;
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
					addVertices(&objFile, vertices, 0.0);
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
			addVertices(&objFile, vertices, 0.0);
			for (int i = 0; i < 4; i++)
				faces.push_back(vertIndex + i);

			vertIndex += 4;
			buildingPolygon = false;
		}
	}

	for (int y = 0; y < image->height; y++)
	{
		for (int x = 0; x < image->width; x++)
		{
			if (*getPixel<uint8>(x, y, image) <= 100)
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
			else
			{
				// end of polygon, add it to the file
				if (buildingPolygon)
				{
					addVertices(&objFile, vertices, 0.0);
					addVertices(&objFile, vertices, 1.0);

					faces.push_back(vertIndex + 4);
					faces.push_back(vertIndex + 5);
					faces.push_back(vertIndex + 6);
					faces.push_back(vertIndex + 7);

					faces.push_back(vertIndex + 0);
					faces.push_back(vertIndex + 1);
					faces.push_back(vertIndex + 5);
					faces.push_back(vertIndex + 4);

					faces.push_back(vertIndex + 4);
					faces.push_back(vertIndex + 7);
					faces.push_back(vertIndex + 3);
					faces.push_back(vertIndex + 0);

					faces.push_back(vertIndex + 1);
					faces.push_back(vertIndex + 2);
					faces.push_back(vertIndex + 6);
					faces.push_back(vertIndex + 5);

					faces.push_back(vertIndex + 2);
					faces.push_back(vertIndex + 3);
					faces.push_back(vertIndex + 7);
					faces.push_back(vertIndex + 6);

					vertIndex += 8;
					buildingPolygon = false;
				}
			}
		}

		// end of line while building polygon? add it to obj file
		if (buildingPolygon)
		{
			addVertices(&objFile, vertices, 0.0);
			addVertices(&objFile, vertices, 1.0);

			faces.push_back(vertIndex + 4);
			faces.push_back(vertIndex + 5);
			faces.push_back(vertIndex + 6);
			faces.push_back(vertIndex + 7);

			faces.push_back(vertIndex + 0);
			faces.push_back(vertIndex + 1);
			faces.push_back(vertIndex + 5);
			faces.push_back(vertIndex + 4);

			faces.push_back(vertIndex + 4);
			faces.push_back(vertIndex + 7);
			faces.push_back(vertIndex + 3);
			faces.push_back(vertIndex + 0);

			faces.push_back(vertIndex + 1);
			faces.push_back(vertIndex + 2);
			faces.push_back(vertIndex + 6);
			faces.push_back(vertIndex + 5);

			faces.push_back(vertIndex + 2);
			faces.push_back(vertIndex + 3);
			faces.push_back(vertIndex + 7);
			faces.push_back(vertIndex + 6);

			vertIndex += 8;
			buildingPolygon = false;
		}
	}


	for (size_t i = 0; i < faces.size(); i = i + 4)
		addFace(&objFile, faces[i], faces[i+1], faces[i+2], faces[i+3]);

	objFile.close();
	return 0;
}
