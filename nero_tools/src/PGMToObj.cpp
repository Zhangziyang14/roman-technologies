#include "ros/ros.h"
#include "fstream"
#include "yaml-cpp/yaml.h"
#include "image_server/OpenCVTools.h"

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

	if (argc != 2)
	{
		ROS_ERROR("Invalid number of arguments. Usage: rosrun nero_tools PGMToObj <filename>.yaml");
		return 0;
	}

	std::ofstream objFile("out.obj");
	std::ifstream fin(argv[1]);
	if (fin.fail())
	{
		ROS_ERROR("Failed to open YAML file.");
		return -1;
	}

	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	double origin_x, origin_y;
	origin_x = origin_y = 0.0;
	double resolution = 0.0;
	std::string mapname = "";

	try
	{
		doc["resolution"] >> resolution;
	} catch (YAML::InvalidScalar)
	{
		ROS_ERROR("No resolution found in YAML file.");
		return -1;
	}
	try
	{
		doc["origin"][0] >> origin_x;
	} catch (YAML::InvalidScalar)
	{
		ROS_ERROR("No origin.x found in YAML file.");
		return -1;
	}
	try
	{
		doc["origin"][1] >> origin_y;
	} catch (YAML::InvalidScalar)
	{
		ROS_ERROR("No origin.y found in YAML file.");
		return -1;
	}
	try
	{
		doc["image"] >> mapname;
	} catch (YAML::InvalidScalar)
	{
		ROS_ERROR("No image name found in YAML file.");
		return -1;
	}

	cv::Mat image = cv::imread(mapname.c_str(), -1);
	if (image.data == NULL)
	{
		std::cout << "It failed.." << std::endl;
		return -1;
	}

	std::vector<uint32_t> faces;

	uint32_t vertIndex = 1;
	cv::Point2d vertices[4]; // left-top, left-bottom, right-bottom, right-top
	bool buildingPolygon = false;
	for (int y = 0; y < image.rows; y++)
	{
		for (int x = 0; x < image.cols; x++)
		{
			//if (*getPixel<uint8>(x, y, image) >= 254)
			if (image.at<uint8_t>(y, x) >= 254)
			{
				if (buildingPolygon == false)
				{
					// if we are starting a line, fill the leftmost vertices
					vertices[0].x = origin_x + x * resolution;
					vertices[0].y = origin_y + (image.rows - y - 1) * resolution;
					vertices[1].x = origin_x + x * resolution;
					vertices[1].y = origin_y + (image.rows - y - 1) * resolution + resolution;
				}

				// fill the rightmost vertices regardless of extending or starting a new line
				vertices[2].x = origin_x + x * resolution + resolution;
				vertices[2].y = origin_y + (image.rows - y - 1) * resolution + resolution;
				vertices[3].x = origin_x + x * resolution + resolution;
				vertices[3].y = origin_y + (image.rows - y - 1) * resolution;

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

	for (int y = 0; y < image.rows; y++)
	{
		for (int x = 0; x < image.cols; x++)
		{
			//if (*getPixel<uint8>(x, y, image) <= 100)
			if (image.at<uint8_t>(y, x) <= 100)
			{
				if (buildingPolygon == false)
				{
					// if we are starting a line, fill the leftmost vertices
					vertices[0].x = origin_x + x * resolution;
					vertices[0].y = origin_y + (image.rows - y - 1) * resolution;
					vertices[1].x = origin_x + x * resolution;
					vertices[1].y = origin_y + (image.rows - y - 1) * resolution + resolution;
				}

				// fill the rightmost vertices regardless of extending or starting a new line
				vertices[2].x = origin_x + x * resolution + resolution;
				vertices[2].y = origin_y + (image.rows - y - 1) * resolution + resolution;
				vertices[3].x = origin_x + x * resolution + resolution;
				vertices[3].y = origin_y + (image.rows - y - 1) * resolution;

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
