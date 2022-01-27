#include "roomProcessor.h"

voxelfield::voxelfield(helperCluster* cluster)
{
	// compute generic voxelfield data
	anchor_ = cluster->getLllPoint();
	gp_Pnt urrPoints= cluster->getUrrPoint();

	xRange_ = urrPoints.X() - anchor_.X();
	yRange_ = urrPoints.Y() - anchor_.Y();
	zRange_ = urrPoints.Z() - anchor_.Z();

	xRelRange_ = ceil(xRange_ / desiredVSize_);
	yRelRange_ = ceil(yRange_ / desiredVSize_);
	zRelRange_ = ceil(zRange_ / desiredVSize_);

	double voxelSizeX = xRange_ / xRelRange_;
	double voxelSizeY = yRange_ / yRelRange_;
	double voxelSizeZ = zRange_ / zRelRange_;

	voxelSize_ = { voxelSizeX, voxelSizeY, voxelSizeZ };


	std::cout << xRange_ << std::endl;
	std::cout << yRange_ << std::endl;
	std::cout << zRange_ << std::endl;

	std::cout << xRelRange_ << std::endl;
	std::cout << yRelRange_ << std::endl;
	std::cout << zRelRange_ << std::endl;

}
