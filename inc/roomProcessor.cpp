#include "roomProcessor.h"

template<typename T>
T voxelfield::linearToRelative(int i) {
	double x = i % xRelRange_;
	double z = i / (xRelRange_ * yRelRange_) - i / (xRelRange_ * yRelRange_) % 1;
	double y = (i - x) / xRelRange_ - z * yRelRange_;

	return T(x, z, y);
}

BoostPoint3D voxelfield::relPointToWorld(BoostPoint3D p)
{
	float xCoord = anchor_.X() + (bg::get<0>(p) * voxelSize_);
	float yCoord = anchor_.Y() + (bg::get<1>(p) * voxelSize_);
	float zCoord = anchor_.Z() + (bg::get<2>(p) * voxelSize_);

	return BoostPoint3D(xCoord, yCoord, zCoord);
}

BoostPoint3D voxelfield::relPointToWorld(int px, int py, int pz)
{
	float xCoord = px * voxelSize_ + voxelSize_ / 2;
	float yCoord = py * voxelSize_ + voxelSize_ / 2;
	float zCoord = pz * voxelSize_ + voxelSize_ / 2;

	return BoostPoint3D(xCoord, yCoord, zCoord);
}

voxelfield::voxelfield(helperCluster* cluster)
{
	// compute generic voxelfield data
	anchor_ = cluster->getLllPoint();
	gp_Pnt urrPoints= cluster->getUrrPoint();

	xRange_ = urrPoints.X() - anchor_.X();
	yRange_ = urrPoints.Y() - anchor_.Y();
	zRange_ = urrPoints.Z() - anchor_.Z();

	xRelRange_ = ceil(xRange_ / voxelSize_);
	yRelRange_ = ceil(yRange_ / voxelSize_);
	zRelRange_ = ceil(zRange_ / voxelSize_);

	totalVoxels_ = xRelRange_ * yRelRange_ * zRelRange_;
	Assignment = std::vector<int>(totalVoxels_, 0);

	if (false)
	{
		std::cout << "cluster debug:" << std::endl;

		std::cout << anchor_.X() << std::endl;
		std::cout << anchor_.Y() << std::endl;
		std::cout << anchor_.Z() << std::endl;


		std::cout << xRange_ << std::endl;
		std::cout << yRange_ << std::endl;
		std::cout << zRange_ << std::endl;

		std::cout << xRelRange_ << std::endl;
		std::cout << yRelRange_ << std::endl;
		std::cout << zRelRange_ << std::endl;
	}
}

void voxelfield::makeRooms(helperCluster* cluster)
{
	int cSize = cluster->getSize();

	for (size_t i = 0; i < totalVoxels_; i++)
	{
		auto midPoint = relPointToWorld(linearToRelative<BoostPoint3D>(i));
		voxel boxel(midPoint, voxelSize_);

		auto boxelGeo = boxel.getVoxelGeo();

		std::vector<Value> qResult;

		auto mi = boxelGeo.min_corner();
		auto m2 = boxelGeo.max_corner();
		//std::cout << bg::get<0>(mi) << ", " << bg::get<1>(mi) << ", " << bg::get<2>(mi) << ", " << std::endl;
		//std::cout << bg::get<0>(m2) << ", " << bg::get<1>(m2) << ", " << bg::get<2>(m2) << ", " << std::endl;

		// find potential intersecting objects
		for (size_t i = 0; i < cSize; i++)
		{
			auto indx = cluster->getHelper(i)->getIndexPointer();
			indx->query(bgi::intersects(boxelGeo), std::back_inserter(qResult));
		}

		// check for actual intersection 

	}
}

voxel::voxel(BoostPoint3D center, double size)
{
	size_ = size;
	center_ = center;
}

bg::model::box<BoostPoint3D> voxel::getVoxelGeo()
{
	float offset = size_ / 2;

	bg::model::point<float, 3, bg::cs::cartesian > lll (bg::get<0>(center_) - offset, bg::get<1>(center_) - offset, bg::get<2>(center_) - offset);
	bg::model::point<float, 3, bg::cs::cartesian > urr(bg::get<0>(center_) + offset, bg::get<1>(center_) + offset, bg::get<2>(center_) + offset);
	
	return bg::model::box<BoostPoint3D>(lll, urr);
}
