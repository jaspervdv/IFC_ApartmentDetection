#include "roomProcessor.h"

template<typename T>
T voxelfield::linearToRelative(int i) {
	double x = i % xRelRange_;
	double z = round(i / (xRelRange_ * yRelRange_)) - round(i / (xRelRange_ * yRelRange_) % 1);
	double y = (i - x) / xRelRange_ - z * yRelRange_;

	return T(x, y, z);
}
std::vector<int> voxelfield::getNeighbours(int voxelIndx)
{
	std::vector<int> neightbours;
	gp_Pnt loc3D = linearToRelative<gp_Pnt>(voxelIndx);

	bool xSmall = loc3D.X() - 1 >= 0;
	bool xBig = loc3D.X() + 1 <= xRelRange_;

	bool ySmall = loc3D.Y() - 1 >= 0;
	bool yBig = loc3D.Y() + 1 <= yRelRange_;

	bool zSmall = loc3D.Z() - 1 >= 0;
	bool zBig = loc3D.Z() + 1 <= zRelRange_;

	// connectivity
	if (xSmall) 
	{ 
		neightbours.emplace_back(voxelIndx - 1); 
		if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ - 1); }
		if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ - 1); }
	}
	if (xBig) 
	{ 
		neightbours.emplace_back(voxelIndx + 1); 
		if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ + 1); }
		if(zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ + 1); }
	}

	if (ySmall)
	{ 
		neightbours.emplace_back(voxelIndx - xRelRange_); 
		if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ - xRelRange_); }
		if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ - xRelRange_); }
	}
	if (yBig) 
	{ 
		neightbours.emplace_back(voxelIndx + xRelRange_); 
		if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ + xRelRange_); }
		if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ + xRelRange_); }
	}

	if (zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)); }
	if (zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_ )); }

	if (xSmall && ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ - 1); }
	if (xBig && yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ + 1); }

	if (xBig && ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ + 1); }
	if (xSmall && yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ - 1); }

	return neightbours;
}
BoostPoint3D voxelfield::relPointToWorld(BoostPoint3D p)

{
	double xCoord = anchor_.X() + (bg::get<0>(p) * voxelSize_);
	double yCoord = anchor_.Y() + (bg::get<1>(p) * voxelSize_);
	double zCoord = anchor_.Z() + (bg::get<2>(p) * voxelSize_);

	return BoostPoint3D(xCoord, yCoord, zCoord);
}

BoostPoint3D voxelfield::relPointToWorld(int px, int py, int pz)
{
	double xCoord = px * voxelSize_ + voxelSize_ / 2;
	double yCoord = py * voxelSize_ + voxelSize_ / 2;
	double zCoord = pz * voxelSize_ + voxelSize_ / 2;

	return BoostPoint3D(xCoord, yCoord, zCoord);
}

void voxelfield::outputFieldToFile()
{
	std::ofstream storageFile;
	storageFile.open("D:/Documents/Uni/Thesis/sources/Models/exports/voxels.txt", std::ios_base::app);
	for (auto it = VoxelLookup.begin(); it != VoxelLookup.end(); ++ it )
	{
		std::vector<gp_Pnt> pointList =  it->second.getCornerPoints(planeRotation_);

		for (size_t k = 0; k < pointList.size(); k++)
		{
			storageFile << pointList[k].X() << ", " << pointList[k].Y() << ", " << pointList[k].Z() << std::endl;
		}

		auto products = it->second.getProducts();
		storageFile << products.size() << std::endl;

		storageFile << "\n";
	}



	storageFile << -planeRotation_;
	storageFile.close();
}

voxelfield::voxelfield(helperCluster* cluster)
{
	// compute generic voxelfield data
	anchor_ = cluster->getLllPoint();
	gp_Pnt urrPoints= cluster->getUrrPoint();

	double xRange = urrPoints.X() - anchor_.X();
	double yRange = urrPoints.Y() - anchor_.Y();
	double zRange = urrPoints.Z() - anchor_.Z();

	xRelRange_ = (int) ceil(xRange / voxelSize_) + 1;
	yRelRange_ = (int) ceil(yRange / voxelSize_) + 1;
	zRelRange_ = (int) ceil(zRange / voxelSize_) + 1;

	totalVoxels_ = xRelRange_ * yRelRange_ * zRelRange_;
	Assignment = std::vector<int>(totalVoxels_, 0);

	planeRotation_ = cluster->getDirection();

	if (false)
	{
		std::cout << "cluster debug:" << std::endl;

		std::cout << anchor_.X() << std::endl;
		std::cout << anchor_.Y() << std::endl;
		std::cout << anchor_.Z() << std::endl;


		std::cout << xRange << std::endl;
		std::cout << yRange << std::endl;
		std::cout << zRange << std::endl;

		std::cout << xRelRange_ << std::endl;
		std::cout << yRelRange_ << std::endl;
		std::cout << zRelRange_ << std::endl;

		std::cout << totalVoxels_ << std::endl;
	}
}

void voxelfield::makeRooms(helperCluster* cluster)
{
	int cSize = cluster->getSize();

	std::vector<helper*> helperList;

	for (size_t i = 0; i < cluster->getSize(); i++)
	{
		helperList.emplace_back(cluster->getHelper(i));
	}

	for (int i = 0; i < totalVoxels_; i++)
	{
		auto midPoint = relPointToWorld(linearToRelative<BoostPoint3D>(i));
		voxel boxel(midPoint, voxelSize_);

		auto boxelGeo = boxel.getVoxelGeo();

		std::vector<Value> qResult;

		// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
		std::vector<gp_Pnt> pointList = boxel.getCornerPoints(planeRotation_);

		// find potential intersecting objects
		for (int j = 0; j < cSize; j++)
		{
			cluster->getHelper(j)->getIndexPointer()->query(bgi::intersects(boxelGeo), std::back_inserter(qResult));

			if (qResult.size() == 0) { continue; }

			for (size_t k = 0; k < qResult.size(); k++)
			{
				IfcSchema::IfcProduct* product = qResult[k].second;

				if (!product->hasRepresentation()) { continue; }
				if (boxel.checkIntersecting(product, helperList[j], pointList))
				{
					Assignment[i] = -1;
					boxel.addProduct(product);
				}
			}
		}
		if (boxel.getIsIntersecting())
		{
			VoxelLookup.emplace(i, boxel);
		}
	}

	for (size_t i = 0; i < totalVoxels_; i++)
	{

	}


	outputFieldToFile();

}

voxel::voxel(BoostPoint3D center, double size)
{
	size_ = size;
	center_ = center;

	gp_Pnt minPoint(bg::get<0>(center) - 1 / 2 * size, bg::get<1>(center) - 1 / 2 * size, bg::get<2>(center) - 1 / 2 * size);
	gp_Pnt maxPoint(bg::get<0>(center) + 1 / 2 * size, bg::get<1>(center) + 1 / 2 * size, bg::get<2>(center) + 1 / 2 * size);
}

bg::model::box<BoostPoint3D> voxel::getVoxelGeo()
{
	double offset = size_ / 2;

	BoostPoint3D lll (bg::get<0>(center_) - offset, bg::get<1>(center_) - offset, bg::get<2>(center_) - offset);
	BoostPoint3D urr(bg::get<0>(center_) + offset, bg::get<1>(center_) + offset, bg::get<2>(center_) + offset);
	
	return bg::model::box<BoostPoint3D>(lll, urr);
}

std::vector<gp_Pnt> voxel::getCornerPoints(double angle)
{
	auto boxelGeo = getVoxelGeo();

	// move to real world space
	auto minPoint = rotatePointWorld(Point3DBTO(boxelGeo.min_corner()), angle);
	auto maxPoint = rotatePointWorld(Point3DBTO(boxelGeo.max_corner()), angle);

	// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
	std::vector<gp_Pnt> pointList;
	pointList.emplace_back(minPoint);
	pointList.emplace_back(maxPoint.X(), minPoint.Y(), minPoint.Z());
	pointList.emplace_back(maxPoint.X(), maxPoint.Y(), minPoint.Z());
	pointList.emplace_back(minPoint.X(), maxPoint.Y(), minPoint.Z());
	pointList.emplace_back(maxPoint);
	pointList.emplace_back(maxPoint.X(), minPoint.Y(), maxPoint.Z());
	pointList.emplace_back(minPoint.X(), minPoint.Y(), maxPoint.Z());
	pointList.emplace_back(minPoint.X(), maxPoint.Y(), maxPoint.Z());

	return pointList;
}

std::vector<std::vector<int>> voxel::getVoxelTriangles()
{
	return {
	{ 0, 1, 5 }, // side	
	{ 0, 5, 6 },
	{ 1, 2, 4 },
	{ 1, 4, 5 },
	{ 2, 3, 7 },
	{ 2, 7, 4 },
	{ 3, 0, 6 },
	{ 3, 6, 7 },
	{ 6, 5, 4 }, // top
	{ 6, 4, 7 },
	{ 0, 3, 2 }, // buttom
	{ 0, 2, 1 }
	};
}

std::vector<std::vector<int>> voxel::getVoxelFaces()
{
	return {
	{ 0, 1, 5, 6 }, // side	
	{ 1, 2, 4, 5 },
	{ 2, 3, 7, 4 },
	{ 3, 0, 6, 7 },
	{ 6, 5, 4, 7 }, // top
	{ 0, 3, 2, 1 }, // buttom
	};
}

double voxel::tVolume(gp_Pnt p, const std::vector<gp_Pnt> vertices) {
	BoostPoint3D p1(vertices[0].X() - vertices[1].X(), vertices[0].Y() - vertices[1].Y(), vertices[0].Z() - vertices[1].Z());
	BoostPoint3D p2(vertices[1].X() - p.X(), vertices[1].Y() - p.Y(), vertices[1].Z() - p.Z());
	BoostPoint3D p3(vertices[2].X() - p.X(), vertices[2].Y() - p.Y(), vertices[2].Z() - p.Z());

	return bg::dot_product(p1, bg::cross_product(p2, p3))/6;
}

bool voxel::checkIntersecting(const IfcSchema::IfcProduct* product, helper* h, std::vector<gp_Pnt> voxelPoints)
{
	std::vector<TopoDS_Face> faceList = h->getObjectFaces(product);
	std::vector<TopoDS_Face> voxelFaceList;

	std::vector<std::vector<int>> vets = getVoxelFaces();

	for (size_t i = 0; i < faceList.size(); i++)
	{
		BRepMesh_IncrementalMesh faceMesh = BRepMesh_IncrementalMesh(faceList[i], 0.004);

		TopLoc_Location loc;
		auto mesh = BRep_Tool::Triangulation(faceList[i], loc);
		const gp_Trsf& trsf = loc.Transformation();

		for (size_t j = 1; j <= mesh.get()->NbTriangles(); j++)
		{
			const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

			std::vector<gp_Pnt> triangePoints;
			for (size_t k = 1; k <= 3; k++)
			{
				gp_Pnt p = mesh->Nodes().Value(theTriangle(k));
				triangePoints.emplace_back(p.Transformed(trsf));
			}

			for (size_t k = 0; k < vets.size(); k++)
			{
				if (checkIntersecting({ voxelPoints[vets[k][0]], voxelPoints[vets[k][1]] }, triangePoints)) { return true; }
				if (checkIntersecting({ voxelPoints[vets[k][1]], voxelPoints[vets[k][2]] }, triangePoints)) { return true; }
				if (checkIntersecting({ voxelPoints[vets[k][2]], voxelPoints[vets[k][3]] }, triangePoints)) { return true; }
				if (checkIntersecting({ voxelPoints[vets[k][3]], voxelPoints[vets[k][0]] }, triangePoints)) { return true; }
			}
		}
	}
	return false;
}

bool voxel::checkIntersecting(const std::vector<gp_Pnt> line, const std::vector<gp_Pnt> triangle)
{
	// check for potential intersection
	double left = tVolume(triangle[0], { triangle[2], line[0], line[1] });
	double right = tVolume(triangle[1], { triangle[2], line[0], line[1] });

	double left2 = tVolume(triangle[1], { triangle[0], line[0], line[1] });
	double right2 = tVolume(triangle[2], { triangle[0], line[0], line[1] });

	double left3 = tVolume(triangle[2], { triangle[1], line[0], line[1] });
	double right3 = tVolume(triangle[0], { triangle[1], line[0], line[1] });


	if (left > 0 && right > 0 || left < 0 && right < 0) { return false; }
	if (left2 > 0 && right2 > 0 || left2 < 0 && right2 < 0) { return false; }
	if (left3 > 0 && right3 > 0 || left3 < 0 && right3 < 0) { return false; }

	double leftFinal = tVolume(line[0], triangle);
	double rightFinal = tVolume(line[1], triangle);

	if (leftFinal > 0 && rightFinal < 0 || rightFinal > 0 && leftFinal < 0)
	{
		isIntersecting_ = true;
		return true;
	}
	return false;
}
