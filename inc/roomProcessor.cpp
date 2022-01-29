#include "roomProcessor.h"

bg::model::point<float, 3, bg::cs::cartesian> voxelfield::relPointToWorld(bg::model::point<float, 3, bg::cs::cartesian> p)
{
	float xCoord = bg::get<0>(p) * voxelSize_ + voxelSize_ / 2;
	float yCoord = bg::get<1>(p) * voxelSize_ + voxelSize_ / 2;
	float zCoord = bg::get<2>(p) * voxelSize_ + voxelSize_ / 2;

	return bg::model::point<float, 3, bg::cs::cartesian>(xCoord, yCoord, zCoord);
}

bg::model::point<float, 3, bg::cs::cartesian> voxelfield::relPointToWorld(int px, int py, int pz)
{
	float xCoord = px * voxelSize_ + voxelSize_ / 2;
	float yCoord = py * voxelSize_ + voxelSize_ / 2;
	float zCoord = pz * voxelSize_ + voxelSize_ / 2;

	return bg::model::point<float, 3, bg::cs::cartesian>(xCoord, yCoord, zCoord);
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

	if (false)
	{
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

	//std::vector<bgi::rtree<std::pair<bg::model::box<bg::model::point<float, 3, bg::cs::cartesian>>, const IfcSchema::IfcProduct*>, bgi::rstar<25>>*> indexRef;

	//for (size_t i = 0; i < cluster->getSize(); i++) { indexRef.emplace_back(cluster->getHelper(i)->getIndexPointer()); }

	std::cout << xRelRange_ << std::endl;
	std::cout << yRelRange_ << std::endl;
	std::cout << zRelRange_ << std::endl;

	for (size_t i = 0; i < totalVoxels_; i++)
	{
		double x = i % xRelRange_;
		double z = i / (xRelRange_ * yRelRange_) - i / (xRelRange_ * yRelRange_) % 1;
		double y =  (i  - x) / xRelRange_ - z * yRelRange_;
	}


	for (size_t i = 0; i < xRelRange_; i++)
	{
		for (size_t j = 0; j < yRelRange_; j++)
		{
			for (size_t k = 0; k < zRelRange_; k++)
			{
				auto midPoint = relPointToWorld(i, j, k);
				voxel boxel(midPoint, voxelSize_);

				auto boxelGeo = boxel.getVoxelGeo();


				//TODO make query
			}
		}
	}


}

voxel::voxel(bg::model::point<float, 3, bg::cs::cartesian> center, double size)
{
	size_ = size;
	center_ = center;
}

bg::model::box<bg::model::point<float, 3, bg::cs::cartesian>> voxel::getVoxelGeo()
{
	float offset = size_ / 2;

	bg::model::point<float, 3, bg::cs::cartesian > lll (bg::get<0>(center_) - offset, bg::get<1>(center_) - offset, bg::get<2>(center_) - offset);
	bg::model::point<float, 3, bg::cs::cartesian > urr(bg::get<0>(center_) + offset, bg::get<1>(center_) + offset, bg::get<2>(center_) + offset);
	
	return bg::model::box<bg::model::point<float, 3, bg::cs::cartesian>>(lll, urr);
}
