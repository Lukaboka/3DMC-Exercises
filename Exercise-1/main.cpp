#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold


	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0; // 2 * (width - 1) * (height - 1) - invalid triangles

	Vector3f* validFaces = new Vector3f[2 * (width - 1) * (height - 1)]; // index positions of vertices of valid faces; 2 triangles per square

	for (int y = 0; y < height - 1; y++)
	{
		for (int x = 0; x < width - 1; x++)
		{

			// 132 and 342 each form a triangle
			Vector3f vertex1 = Vector3f(vertices[x + (y * width)].position.x(), vertices[x + (y * width)].position.y(), vertices[x + (y * width)].position.z()); // x, y
			Vector3f vertex2 = Vector3f(vertices[x + 1 + (y * width)].position.x(), vertices[x + 1 + (y * width)].position.y(), vertices[x + 1 + (y * width)].position.z()); // x + 1, y
			Vector3f vertex3 = Vector3f(vertices[x + ((y + 1) * width)].position.x(), vertices[x + ((y + 1) * width)].position.y(), vertices[x + ((y + 1) * width)].position.z()); // x, y + 1
			Vector3f vertex4 = Vector3f(vertices[x + 1 + ((y + 1) * width)].position.x(), vertices[x + 1 + ((y + 1) * width)].position.y(), vertices[x + 1 + ((y + 1) * width)].position.z()); // x + 1, y + 1

			// check if any of the vertices of current face is invalid
			if (vertex1 == Vector3f(MINF, MINF, MINF) 
				|| vertex2 == Vector3f(MINF, MINF, MINF)
				|| vertex3 == Vector3f(MINF, MINF, MINF))
			{
				validFaces[2 * (x + (y * (width - 1)))] = Vector3f(MINF, MINF, MINF);
			}

			// check if the edges of current face are below threshold
			else if ((vertex2 - vertex1).norm() < edgeThreshold
				|| (vertex2 - vertex3).norm() < edgeThreshold
				|| (vertex1 - vertex3).norm() < edgeThreshold)
			{
				validFaces[2 * (x + (y * (width - 1)))] = Vector3f(MINF, MINF, MINF);
			} 

			// note down vertex indices if face is valid
			else
			{
				validFaces[2 * (x + (y * (width - 1)))] = Vector3f(x + (y * width), x + ((y + 1) * width), x + 1 + (y * width)); // triangle 132
				nFaces++;
			}

			/*-----------------------------------------------------------------------*/

			// check if any of the vertices of next face is invalid
			if (vertex3 == Vector3f(MINF, MINF, MINF) 
				|| vertex4 == Vector3f(MINF, MINF, MINF)
				|| vertex2 == Vector3f(MINF, MINF, MINF))
			{
				validFaces[2 * (x + (y * (width - 1))) + 1] = Vector3f(MINF, MINF, MINF);
			}

			// check if the edges of next face are below threshold
			else if ((vertex4 -vertex3).norm() < edgeThreshold
				|| (vertex3 - vertex2).norm() < edgeThreshold
				|| (vertex2 - vertex4).norm() < edgeThreshold)
			{
				validFaces[2 * (x + (y * (width - 1))) + 1] = Vector3f(MINF, MINF, MINF);
			}

			// note down vertex indices if face is valid
			else 
			{
				validFaces[2 * (x + (y * (width - 1))) + 1] =  Vector3f(x + ((y + 1) * width), x + 1 + ((y + 1) * width), x + 1 + (y * width)); // triangle 342
				nFaces++;
			}
		}
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	outFile << "# list of vertices" << std::endl;

	outFile << "# X Y Z R G B A" << std::endl;

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (vertices[x + (y * width)].position.x() == MINF)
			{
				outFile << "0 0 0 0 0 0 0" << std::endl;
			}
			else
			{
				outFile << vertices[x + (y * width)].position.x() << " " << vertices[x + (y * width)].position.y()
					<< " " << vertices[x + (y * width)].position.z() << " " << (int) vertices[x + (y * width)].color.x() << " "
					<< (int) vertices[x + (y * width)].color.y() << " " << (int) vertices[x + (y * width)].color.z() << " "
					<< (int) vertices[x + (y * width)].color.w() << std::endl;
			}
		}
	}
	// TODO: save valid faces
	outFile << "# list of faces" << std::endl;
	outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;

	for (int y = 0; y < height - 1; y++)
	{
		for (int x = 0; x < width - 1; x++)
		{
			if (validFaces[2 * (x + (y * (width - 1)))] != Vector3f(MINF, MINF, MINF))
			{
				outFile << "3 " << validFaces[2 * (x + (y * (width - 1)))].x() << " " 
					<< validFaces[2 * (x + (y * (width - 1)))].y() << " " 
					<< validFaces[2 * (x + (y * (width - 1)))].z() << std::endl;
			}
			if (validFaces[2 * (x + (y * (width - 1))) + 1] != Vector3f(MINF, MINF, MINF))
			{
				outFile << "3 " << validFaces[2 * (x + (y * (width - 1))) + 1].x() << " "
					<< validFaces[2 * (x + (y * (width - 1))) + 1].y() << " "
					<< validFaces[2 * (x + (y * (width - 1))) + 1].z() << std::endl;
			}
		}
	}


	// close file
	outFile.close();

	delete[] validFaces;

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "D:/Uni/3DScanning&MotionCapture/Exercises/Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap

		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		for (int y = 0; y < sensor.GetDepthImageHeight(); y++)
		{
			for (int x = 0; x < sensor.GetDepthImageWidth(); x++)
			{
				float curDepth = depthMap[x + (y * sensor.GetDepthImageWidth())];
				if (curDepth == MINF)
				{
					vertices[x + (y * sensor.GetDepthImageWidth())].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[x + (y * sensor.GetDepthImageWidth())].color = Vector4uc(0, 0, 0, 0);
				}
				else
				{
					Vector3f vertexCamera = depthIntrinsicsInv * Vector3f(x * curDepth, y * curDepth, curDepth);
					Vector4f vertexWorld = depthExtrinsicsInv * Vector4f(vertexCamera.x(), vertexCamera.y(), vertexCamera.z(), 1.0);

					vertices[x + (y * sensor.GetDepthImageWidth())].position = vertexWorld;
					vertices[x + (y * sensor.GetDepthImageWidth())].color = Vector4uc(colorMap[x * 4 + (y * sensor.GetDepthImageWidth())], 
						colorMap[(x * 4) + 1 + (y * sensor.GetDepthImageWidth())], colorMap[(x * 4) + 2 + (y * sensor.GetDepthImageWidth())], 
						colorMap[(x * 4) + 3 + (y * sensor.GetDepthImageWidth())]);
				}
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}