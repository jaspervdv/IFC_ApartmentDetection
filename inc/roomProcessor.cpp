#include "roomProcessor.h"

template<typename T>
T voxelfield::linearToRelative(int i) {
	double x = i % xRelRange_;
	double z = round(i / (xRelRange_ * yRelRange_)) - round(i / (xRelRange_ * yRelRange_) % 1);
	double y = (i - x) / xRelRange_ - z * yRelRange_;

	return T(x, y, z);
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

	if (true)
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
				IfcSchema::IfcProduct* product = qResult[j].second;

				if (!product->hasRepresentation()) { continue; }
				if (boxel.checkIntersecting(product, helperList[j], pointList))
				{
					Assignment[i] = -1;
					// TODO internalize intersecting data
				}
			}
		}
		if (boxel.getIsIntersecting())
		{
			VoxelLookup.emplace(i, boxel);
			// TODO store product pointer
		}
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
	auto minPoint = rotatePointWorld(Point3DBTO(boxelGeo.min_corner()), -angle);
	auto maxPoint = rotatePointWorld(Point3DBTO(boxelGeo.max_corner()), -angle);

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

double voxel::tVolume(gp_Pnt p, const std::vector<gp_Pnt> vertices) {
	BoostPoint3D p1(vertices[0].X() - vertices[1].X(), vertices[0].Y() - vertices[1].Y(), vertices[0].Z() - vertices[1].Z());
	BoostPoint3D p2(vertices[1].X() - p.X(), vertices[1].Y() - p.Y(), vertices[1].Z() - p.Z());
	BoostPoint3D p3(vertices[2].X() - p.X(), vertices[2].Y() - p.Y(), vertices[2].Z() - p.Z());

	return bg::dot_product(p1, bg::cross_product(p2, p3))/6;
}

bool voxel::checkIntersecting(const IfcSchema::IfcProduct* product, helper* h, std::vector<gp_Pnt> voxelPoints)
{
	std::vector<gp_Pnt> productPointList = h->getObjectPoints(product, true);

	for (size_t i = 0; i < productPointList.size(); i+=2)
	{
		gp_Pnt p1 = productPointList[i];
		gp_Pnt p2 = productPointList[i + 1];

		// check for potential intersection
		for (size_t j = 0; j < triangulationBoxel.size(); j++)
		{
			for (size_t k = 0; k < 3; k++)
			{
				std::vector<int> ar = { 0, 1, 2 };
				for (int h = 0; h < 3; h++)
				{ 
					if (ar[h] == k) { ar.erase(ar.begin() + h); } 
				}

				double left = tVolume(voxelPoints[triangulationBoxel[j][ar[0]]], { voxelPoints[triangulationBoxel[j][k]], p1, p2 });
				double right = tVolume(voxelPoints[triangulationBoxel[j][ar[1]]], { voxelPoints[triangulationBoxel[j][k]], p1, p2 });

				if (left > 0 && right > 0 || left < 0 && right < 0) 
				{ 
					break; 
				}

				std::vector<gp_Pnt> basePointList = { voxelPoints[triangulationBoxel[j][0]], voxelPoints[triangulationBoxel[j][1]], voxelPoints[triangulationBoxel[j][2]] };

				double leftFinal = tVolume(p1, basePointList);
				double rightFinal = tVolume(p2, basePointList);

				if (leftFinal > 0 && rightFinal < 0 || rightFinal > 0 && leftFinal < 0)
				{
					isIntersecting_ = true;
					return true;
				}
			}
		}
	}
	return false;
}
