#include "roomProcessor.h"


void voxelfield::createGraph(helperCluster* cluster) {
	// update data to outside 
	int cSize = cluster->getSize();

	// create outside object
	roomObject* outsideObject = new roomObject(nullptr, roomObjectList_.size());
	outsideObject->setIsOutSide();

	//roomObjectList.emplace_back(outsideObject);
	roomObjectList_.insert(roomObjectList_.begin(), outsideObject);

	for (size_t i = 0; i < cSize; i++)
	{
		std::vector<ConnectLookupValue> lookup = cluster->getHelper(i)->getFullClookup();


		for (size_t j = 0; j < lookup.size(); j++)
		{
			if (std::get<2>(lookup[j])->size() == 1)
			{
				std::vector<gp_Pnt> doorPointList = cluster->getHelper(i)->getObjectPoints(std::get<0>(lookup[j]));
				
				double lowZ = 99999999999;
				for (size_t k = 0; k < doorPointList.size(); k++)
				{
					double pZ = doorPointList[k].Z();
					if (pZ < lowZ) { lowZ = pZ; }
				}

				if (lowZ < 2 && lowZ > - 0.5) //TODO door height needs to be smarter
				{
					std::get<2>(lookup[j])[0][0]->addConnection(outsideObject);
				}

			}
		}
	}

	// Make sections
	int counter = 0;

	std::vector<roomObject*> bufferList;

	for (size_t i = 0; i < roomObjectList_.size(); i++)
	{

		roomObject* currentRoom = roomObjectList_[i];

		if (!currentRoom->isInside())
		{
			currentRoom->setSNum(-2);
			continue;
		} 
		if (currentRoom->getConnections().size() != 1) { continue; } // always start isolated
		if (currentRoom->getSNum() != -1) { continue; }

		bufferList.emplace_back(currentRoom);
		currentRoom->setSNum(counter);

		while (bufferList.size() > 0)
		{
			std::vector<roomObject*> tempBufferList;
			tempBufferList.clear();

			for (size_t j = 0; j < bufferList.size(); j++)
			{
				std::vector<roomObject*> connections = bufferList[j]->getConnections();
				for (size_t k = 0; k < connections.size(); k++)
				{
					if (connections[k]->getSNum() == - 1 && connections[k]->isInside())
					{
						connections[k]->setSNum(counter);
						tempBufferList.emplace_back(connections[k]);
					}
				}
			}
			bufferList.clear();
			bufferList = tempBufferList;
		}
		counter++;
	}
}


TopoDS_Face makeFace(std::vector<gp_Pnt> voxelPointList, std::vector<int> pointFaceIndx) {
	gp_Pnt p0(voxelPointList[pointFaceIndx[0]]);
	gp_Pnt p1(voxelPointList[pointFaceIndx[1]]);
	gp_Pnt p2(voxelPointList[pointFaceIndx[2]]);
	gp_Pnt p3(voxelPointList[pointFaceIndx[3]]);

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	TopoDS_Wire wire = BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3);
	return BRepBuilderAPI_MakeFace(wire);
}

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
	bool xBig = loc3D.X() + 1 < xRelRange_;

	bool ySmall = loc3D.Y() - 1 >= 0;
	bool yBig = loc3D.Y() + 1 < yRelRange_;

	bool zSmall = loc3D.Z() - 1 >= 0;
	bool zBig = loc3D.Z() + 1 < zRelRange_;

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

	if (xSmall && ySmall && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_) - xRelRange_ - 1 ); }
	if (xBig && ySmall && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)- xRelRange_ + 1); }

	if (xSmall && yBig && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_) + xRelRange_ - 1); }
	if (xBig && yBig && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)+ xRelRange_ + 1); }

	if (xSmall && yBig && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_) + xRelRange_ - 1); }
	if (xBig && yBig && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_) + xRelRange_ + 1); }

	if (xSmall && ySmall && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_) - xRelRange_ - 1); }
	if (xBig && ySmall && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_) - xRelRange_ + 1); }

	return neightbours;
}
BoostPoint3D voxelfield::relPointToWorld(BoostPoint3D p)

{
	double xCoord = anchor_.X() + (bg::get<0>(p) * voxelSize_);
	double yCoord = anchor_.Y() + (bg::get<1>(p) * voxelSize_);
	double zCoord = anchor_.Z() + (bg::get<2>(p) * voxelSizeZ_);

	return BoostPoint3D(xCoord, yCoord, zCoord);
}

BoostPoint3D voxelfield::relPointToWorld(int px, int py, int pz)
{
	double xCoord = px * voxelSize_ + voxelSize_ / 2;
	double yCoord = py * voxelSize_ + voxelSize_ / 2;
	double zCoord = pz * voxelSizeZ_ + voxelSizeZ_ / 2;

	return BoostPoint3D(xCoord, yCoord, zCoord);
}

std::vector<TopoDS_Face> voxelfield::getPartialFaces(std::vector<int> roomIndx, int voxelIndx)
{
	gp_Pnt loc3D = linearToRelative<gp_Pnt>(voxelIndx);

	bool faceLeft = true;
	bool faceRight = true;
	bool faceFront = true;
	bool faceBack = true;
	bool faceUp = true;
	bool faceDown = true;

	// leftFace
	for (size_t i = 0; i < roomIndx.size(); i++)
	{
		if (roomIndx[i] == voxelIndx - 1) { faceLeft = false; }
		if (roomIndx[i] == voxelIndx + 1) { faceRight = false; }
		if (roomIndx[i] == voxelIndx - xRelRange_) { faceFront = false; }
		if (roomIndx[i] == voxelIndx + xRelRange_) { faceBack = false; }
		if (roomIndx[i] == voxelIndx - (xRelRange_) * (yRelRange_)) { faceDown = false; }
		if (roomIndx[i] == voxelIndx + (xRelRange_) * (yRelRange_)) { faceUp = false; }
	}

	if (!faceLeft && !faceRight && !faceFront && !faceBack && !faceUp && !faceDown){ return {};}
	voxel* currentBoxel = VoxelLookup_[voxelIndx];

	std::vector<TopoDS_Face> faceList;
	std::vector<gp_Pnt> voxelPointList = currentBoxel->getCornerPoints(planeRotation_);
	std::vector<std::vector<int>> voxelFaceList = currentBoxel->getVoxelFaces();

	if (faceLeft) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[3])); }
	if (faceRight) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[1])); }
	if (faceFront) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[0])); }
	if (faceBack) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[2])); }
	if (faceUp) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[4])); }
	if (faceDown) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[5])); }

	return faceList;
}

TopoDS_Face voxelfield::getLowestFace(TopoDS_Shape shape)
{
	TopExp_Explorer expl;
	std::vector<TopoDS_Face> faceList;
	double lowestZ = 9999;
	int lowestFaceIndx = -1;

	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) { faceList.emplace_back(TopoDS::Face(expl.Current())); }

	for (int j = 0; j < faceList.size(); j++)
	{
		int aEdges = 0;
		double totalZ = 0;

		for (expl.Init(faceList[j], TopAbs_VERTEX); expl.More(); expl.Next())
		{
			aEdges++;
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			totalZ += p.Z();
		}

		if (lowestZ > (totalZ / aEdges))
		{
			lowestZ = totalZ / aEdges;
			lowestFaceIndx = j;
		}
	}
	return faceList[lowestFaceIndx];
}

std::vector<TopoDS_Face> voxelfield::getRoomFootprint(TopoDS_Shape shape)
{
	TopExp_Explorer expl;
	std::vector<TopoDS_Face> faceList;
	double lowestZ = 9999;
	int lowestFaceIndx = -1;

	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) { faceList.emplace_back(TopoDS::Face(expl.Current())); }

	// get horizontal faces
	std::vector<TopoDS_Face> horizontalFaceList;
	std::vector<double> horizontalFaceHeight;
	double totalHeight = 0;

	for (int j = 0; j < faceList.size(); j++)
	{
		double z = 0;
		int c = 0;
		bool flat = true;

		for (expl.Init(faceList[j], TopAbs_VERTEX); expl.More(); expl.Next()){
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			if (c == 0)
			{
				z = p.Z();
				c++;
				continue;
			}

			if (z > p.Z() + 0.01 || z < p.Z() - 0.01)
			{
				flat = false;
				break;
			}
		}

		if (flat) 
		{ 
			horizontalFaceList.emplace_back(faceList[j]);
			horizontalFaceHeight.emplace_back(z);
			totalHeight += z;
		}
	}

	double averageHeigth = totalHeight / horizontalFaceList.size();

	std::vector<TopoDS_Face> footprint;
	for (size_t i = 0; i < horizontalFaceHeight.size(); i++)
	{
		if (horizontalFaceHeight[i] < averageHeigth)
		{
			footprint.emplace_back(horizontalFaceList[i]);
		}
	}

	return footprint;
}

void voxelfield::addVoxel(int indx, helperCluster* cluster)
{
	int cSize = cluster->getSize();
	auto midPoint = relPointToWorld(linearToRelative<BoostPoint3D>(indx));
	voxel* boxel = new voxel(midPoint, voxelSize_, voxelSizeZ_);

	// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
	auto boxelGeo = boxel->getVoxelGeo();
	std::vector<gp_Pnt> pointList = boxel->getCornerPoints(planeRotation_);

	// find potential intersecting objects
	std::vector<Value> qResult;
	for (int j = 0; j < cSize; j++)
	{
		qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
		cluster->getHelper(j)->getIndexPointer()->query(bgi::intersects(boxelGeo), std::back_inserter(qResult));

		if (qResult.size() == 0) { continue; }

		for (size_t k = 0; k < qResult.size(); k++)
		{
			LookupValue lookup = cluster->getHelper(j)->getLookup(qResult[k].second);
			IfcSchema::IfcProduct* product = std::get<0>(lookup);

			if (!product->hasRepresentation()) { continue; }

			if (boxel->checkIntersecting(lookup, pointList, cluster->getHelper(j)))
			{
				Assignment_[indx] = -1;
				VoxelLookup_.emplace(indx, boxel);
				return;
			}
		}
	}
	VoxelLookup_.emplace(indx, boxel);
}

void voxelfield::outputFieldToFile()
{
	std::ofstream storageFile;
	storageFile.open("D:/Documents/Uni/Thesis/sources/Models/exports/voxels.txt");
	for (auto it = VoxelLookup_.begin(); it != VoxelLookup_.end(); ++ it )
	{
		std::vector<gp_Pnt> pointList = it->second->getCornerPoints(planeRotation_);

		//if (it->second->getRoomNumbers().size() == 0) { continue; }
		//if (!it->second->getIsInside()) { continue; }
		if (!it->second->getIsIntersecting()) { continue; }

		for (size_t k = 0; k < pointList.size(); k++)
		{
			storageFile << pointList[k].X() << ", " << pointList[k].Y() << ", " << pointList[k].Z() << std::endl;
		}

		//storageFile << it->second->getRoomNumbers().back() << std::endl;
		storageFile << "1" << std::endl;

		storageFile << "\n";
	}

	storageFile << -planeRotation_;
	storageFile.close();
}

voxelfield::voxelfield(helperCluster* cluster, bool isFlat)
{
	// ask user for desired voxel dimensions

	std::string stringXYSize = "";
	std::string stringZSize = "";

	while (true)
	{
		std::cout << "Enter voxel XY dimenion (double):";
		std::cin >> stringXYSize;

		char* end = nullptr;
		double val = strtod(stringXYSize.c_str(), &end);


		if (end != stringXYSize.c_str() && *end == '\0' && val != HUGE_VAL)
		{
			voxelSize_ = val;
			break;
		}
	}

	while (true)
	{
		std::cout << "Enter voxel Z dimension (double):";
		std::cin >> stringZSize;

		char* end = nullptr;
		double val = strtod(stringXYSize.c_str(), &end);


		if (end != stringXYSize.c_str() && *end == '\0' && val != HUGE_VAL)
		{
			voxelSizeZ_ = val;
			break;
		}
	}

	std::cout << std::endl;

	// compute generic voxelfield data
	anchor_ = cluster->getLllPoint();
	gp_Pnt urrPoints = cluster->getUrrPoint();

	double xRange = urrPoints.X() - anchor_.X();
	double yRange = urrPoints.Y() - anchor_.Y();
	double zRange = urrPoints.Z() - anchor_.Z();

	xRelRange_ = (int) ceil(xRange / voxelSize_) + 1;
	yRelRange_ = (int) ceil(yRange / voxelSize_) + 1;
	zRelRange_ = (int) ceil(zRange / voxelSizeZ_) + 1;

	totalVoxels_ = xRelRange_ * yRelRange_ * zRelRange_;
	Assignment_ = std::vector<int>(totalVoxels_, 0);

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

void voxelfield::writeGraph(std::string path)
{
	// output the graph data
	std::string p = path;

	std::ofstream storageFile;
	storageFile.open(path);

	storageFile << "_pointList_" << std::endl;
	for (size_t i = 0; i < roomObjectList_.size(); i++)
	{
		gp_Pnt p = roomObjectList_[i]->getPoint();
		storageFile << p.X() << ", " << p.X() << ", " << p.Z() << std::endl;
	}
	storageFile << "_name_" << std::endl;
	for (size_t i = 0; i < roomObjectList_.size(); i++)
	{
		if (roomObjectList_[i]->isInside())
		{
			if (roomObjectList_[i]->getSelf()->hasLongName()) { storageFile << roomObjectList_[i]->getSelf()->LongName() << std::endl; }
			else { storageFile << roomObjectList_[i]->getSelf()->Name() << std::endl; }
		}
		else
		{
			storageFile << "Outside" << std::endl;
		}
	}

	storageFile << "_area_" << std::endl;
	storageFile << 100 << std::endl;
	for (size_t i = 0; i < roomAreaList_.size(); i++)
	{
		storageFile << roomAreaList_[i] << std::endl;
	}

	storageFile << "_connection_" << std::endl;
	for (size_t i = 0; i < roomObjectList_.size(); i++)
	{
		auto connections = roomObjectList_[i]->getConnections();

		for (size_t j = 0; j < connections.size(); j++)
		{
			if (roomObjectList_[i]->getIdx() + 1 >= roomObjectList_.size())
			{
				storageFile << 0 << ", " << connections[j]->getIdx() + 1 << std::endl;
			}
			else if (connections[j]->getIdx() + 1 >= roomObjectList_.size())
			{
				storageFile << roomObjectList_[i]->getIdx() + 1 << ", " << 0 << std::endl;
			}
			else
			{
				storageFile << roomObjectList_[i]->getIdx() + 1 << ", " << connections[j]->getIdx() + 1 << std::endl;
			}

		}
	}

	storageFile << "_sections_" << std::endl;
	for (size_t i = 0; i < roomObjectList_.size(); i++)
	{
		storageFile << roomObjectList_[i]->getSNum() << std::endl;
	}

}

std::vector<std::string> voxelfield::getSemanticMatch(std::vector<IfcSchema::IfcSpace*> semanticSources, double roomNum)
{
	// unload semantic data
	double semanticListLenght = semanticSources.size();

	std::string semanticName = "Automatic Space";
	std::string semanticLongName = "Automatic Space: " + std::to_string(roomNum);
	std::string semanticDescription = "";

	if (semanticListLenght > 0)
	{
		IfcSchema::IfcSpace* matchingSpaceObject = semanticSources[0];
		if (matchingSpaceObject->hasName()) { semanticName = matchingSpaceObject->Name(); }
		if (matchingSpaceObject->hasLongName()) { semanticLongName = matchingSpaceObject->LongName(); }
		if (matchingSpaceObject->hasDescription()) { semanticDescription = matchingSpaceObject->Description(); }

		if (semanticListLenght > 1) // find solution for when multiple semantic data is found
		{
			IfcSchema::IfcSpace* matchingSpaceObject = semanticSources[1];
			if (matchingSpaceObject->hasName()) { semanticName = semanticName + " (" + matchingSpaceObject->Name(); }
			if (matchingSpaceObject->hasLongName()) { semanticLongName = semanticLongName + " (" + matchingSpaceObject->LongName(); }
			if (matchingSpaceObject->hasDescription()) { semanticDescription = semanticDescription + " (" + matchingSpaceObject->Description(); }

			for (size_t j = 2; j < semanticListLenght; j++)
			{
				IfcSchema::IfcSpace* matchingSpaceObject = semanticSources[j];
				if (j == semanticListLenght - 1)
				{
					if (matchingSpaceObject->hasName()) { semanticName = semanticName + " & " + matchingSpaceObject->Name() + ")"; }
					if (matchingSpaceObject->hasLongName()) { semanticLongName = semanticLongName + " & " + matchingSpaceObject->LongName() + ")"; }
					if (matchingSpaceObject->hasDescription()) { semanticDescription = semanticDescription + " & " + matchingSpaceObject->Description() + ")"; }
				}
				else {
					if (matchingSpaceObject->hasName()) { semanticName = semanticName + ", " + matchingSpaceObject->Name(); }
					if (matchingSpaceObject->hasLongName()) { semanticLongName = semanticLongName + ", " + matchingSpaceObject->LongName(); }
					if (matchingSpaceObject->hasDescription()) { semanticDescription = semanticDescription + ", " + matchingSpaceObject->Description(); }
				}
			}
		}
	}

	return { semanticName, semanticLongName, semanticDescription } ;

}

void voxelfield::makeRooms(helperCluster* cluster)
{
	int cSize = cluster->getSize();
	int roomLoc = -1;
	double unitScale = 1;

	// find helper containing the room objects
	for (size_t i = 0; i < cSize; i++)
	{
		if (cluster->getHelper(i)->getHasRoom())
		{
			roomLoc = i;
			unitScale = 1 / cluster->getHelper(i)->getLengthMultiplier();
			break;
		}
	}

	if (roomLoc == -1)
	{
		std::cout << "[WARNING] cannot find valid room storing model" << std::endl;
		return;
	}

	// remove all rell space boundaries
	for (size_t i = 0; i < cSize; i++)
	{
		IfcSchema::IfcRelSpaceBoundary::list::ptr rSBList = cluster->getHelper(i)->getSourceFile()->instances_by_type<IfcSchema::IfcRelSpaceBoundary>();

		for (IfcSchema::IfcRelSpaceBoundary::list::it it = rSBList->begin(); it != rSBList->end(); ++it)
		{
			IfcSchema::IfcRelSpaceBoundary* rSB= *it;
			cluster->getHelper(i)->getSourceFile()->removeEntity(rSB);
		}
	}

	GProp_GProps gprop;
	IfcSchema::IfcProduct::list::ptr roomProducts(new IfcSchema::IfcProduct::list);

	// pre make hierachy helper
	IfcHierarchyHelper<IfcSchema> hierarchyHelper;

	// get storey elevations from file
	IfcSchema::IfcBuildingStorey::list::ptr buildingStoreys = cluster->getHelper(roomLoc)->getSourceFile()->instances_by_type<IfcSchema::IfcBuildingStorey>();
	std::vector<const IfcSchema::IfcBuildingStorey*> storeys;
	std::vector<double> elevations;

	for (IfcSchema::IfcBuildingStorey::list::it it = buildingStoreys->begin(); it != buildingStoreys->end(); ++it)
	{
		const IfcSchema::IfcBuildingStorey* storey = *it;
		storeys.emplace_back(storey);
		elevations.emplace_back(storey->Elevation() * unitScale);
	}

	// test voxel for intersection and add voxel objects to the voxelfield
	std::cout << "[INFO] Populate Grid" << std::endl;
	for (int i = 0; i < totalVoxels_; i++) { 

		if (i%250 == 0)
		{
			std::cout.flush();
			std::cout << i << " of " << totalVoxels_ << "\r";
		}

		addVoxel(i, cluster); 
	}
	std::cout << totalVoxels_ << " of " << totalVoxels_ << std::endl;
	std::cout << std::endl;


	// asign rooms
	int roomnum = 0;
	int temps = 0;
	
	std::cout << "[INFO]Room Growing" << std::endl;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (Assignment_[i] == 0) // Find unassigned voxel
		{
			std::vector<int> totalRoom = growRoom(i, roomnum);
			if (totalRoom.size() == 0) { continue; }

			//std::cout.flush();
			std::cout << "Room nr: " << roomnum + 1 << "\r";

			BRep_Builder brepBuilder;
			BRepBuilderAPI_Sewing brepSewer;

			TopoDS_Shell shell;
			brepBuilder.MakeShell(shell);
			TopoDS_Solid roughRoomShape;
			brepBuilder.MakeSolid(roughRoomShape);

			gp_Pnt inRoomPoint(99999, 99999, 99999);
			bool hasInsidePoint = false;

			// create bbox around rough room shape
			gp_Pnt lll(9999, 9999, 9999);
			gp_Pnt urr(-9999, -9999, -9999);

			gp_Pnt qlll(9999, 9999, 9999);
			gp_Pnt qurr(-9999, -9999, -9999);

			for (size_t j = 1; j < totalRoom.size(); j++)
			{
				voxel* currentBoxel = VoxelLookup_[totalRoom[j]];
				std::vector<gp_Pnt> cornerPoints = currentBoxel->getCornerPoints(planeRotation_);
				std::vector<gp_Pnt> cornerPointsRel = currentBoxel->getCornerPoints(0);
				for (size_t k = 0; k < cornerPoints.size(); k++)
				{
					auto currentCorner = rotatePointWorld(cornerPoints[k], planeRotation_);
					if (urr.X() < currentCorner.X()) { urr.SetX(currentCorner.X()); }
					if (urr.Y() < currentCorner.Y()) { urr.SetY(currentCorner.Y()); }
					if (urr.Z() < currentCorner.Z()) { urr.SetZ(currentCorner.Z()); }
					if (lll.X() > currentCorner.X()) { lll.SetX(currentCorner.X()); }
					if (lll.Y() > currentCorner.Y()) { lll.SetY(currentCorner.Y()); }
					if (lll.Z() > currentCorner.Z()) { lll.SetZ(currentCorner.Z()); }

					auto currentCornerRel = cornerPointsRel[k];
					if (qurr.X() < currentCornerRel.X()) { qurr.SetX(currentCornerRel.X()); }
					if (qurr.Y() < currentCornerRel.Y()) { qurr.SetY(currentCornerRel.Y()); }
					if (qurr.Z() < currentCornerRel.Z()) { qurr.SetZ(currentCornerRel.Z()); }
					if (qlll.X() > currentCornerRel.X()) { qlll.SetX(currentCornerRel.X()); }
					if (qlll.Y() > currentCornerRel.Y()) { qlll.SetY(currentCornerRel.Y()); }
					if (qlll.Z() > currentCornerRel.Z()) { qlll.SetZ(currentCornerRel.Z()); }
				}

				// search for inside point
				if (!currentBoxel->getIsIntersecting() && !hasInsidePoint)
				{
					inRoomPoint = rotatePointWorld(cornerPoints[5], planeRotation_);
					inRoomPoint.SetZ(inRoomPoint.Z() - currentBoxel->getZ() / 2);
					hasInsidePoint = true;
				}
			}

			gp_Pnt p0(rotatePointWorld(lll, -planeRotation_));
			gp_Pnt p1 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()), -planeRotation_);
			gp_Pnt p2 = rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()), -planeRotation_);
			gp_Pnt p3 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()), -planeRotation_);

			gp_Pnt p4(rotatePointWorld(urr, -planeRotation_));
			gp_Pnt p5 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), urr.Z()), -planeRotation_);
			gp_Pnt p6 = rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), urr.Z()), -planeRotation_);
			gp_Pnt p7 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), urr.Z()), -planeRotation_);

			gp_Pnt pC = rotatePointWorld(gp_Pnt(lll.X() + (urr.X() - lll.X()) / 2, lll.Y() + (urr.Y() - lll.Y()) / 2, lll.Z() + (urr.Z() - lll.Z()) / 2), -planeRotation_);
			gp_Pnt pCR = gp_Pnt(qlll.X() + (qurr.X() - qlll.X()) / 2, qlll.Y() + (qurr.Y() - qlll.Y()) / 2, qlll.Z() + (qurr.Z() - qlll.Z()) / 2);

			TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
			TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
			TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
			TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

			TopoDS_Edge edge4 = BRepBuilderAPI_MakeEdge(p4, p5);
			TopoDS_Edge edge5 = BRepBuilderAPI_MakeEdge(p5, p6);
			TopoDS_Edge edge6 = BRepBuilderAPI_MakeEdge(p6, p7);
			TopoDS_Edge edge7 = BRepBuilderAPI_MakeEdge(p7, p4);

			TopoDS_Edge edge8 = BRepBuilderAPI_MakeEdge(p0, p6);
			TopoDS_Edge edge9 = BRepBuilderAPI_MakeEdge(p3, p7);
			TopoDS_Edge edge10 = BRepBuilderAPI_MakeEdge(p2, p4);
			TopoDS_Edge edge11 = BRepBuilderAPI_MakeEdge(p1, p5);

			std::vector<TopoDS_Face> faceList;

			faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3)));
			faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge4, edge5, edge6, edge7)));
			faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge8, edge5, edge11)));
			faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge3, edge9, edge6, edge8)));
			faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge2, edge10, edge7, edge9)));
			faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge1, edge11, edge4, edge10)));

			for (size_t k = 0; k < faceList.size(); k++) { brepSewer.Add(faceList[k]); }

			brepSewer.Perform();
			brepBuilder.Add(roughRoomShape, brepSewer.SewedShape());

			gp_Trsf scaler;
			scaler.SetScale(pC, 1.3);

			// finalize rough room shape
			TopoDS_Shape sizedRoomShape = BRepBuilderAPI_Transform(roughRoomShape, scaler).ModifiedShape(roughRoomShape);
			p0.Transform(scaler);
			p4.Transform(scaler);

			// finalize qbox shape
			scaler.SetScale(pCR, 1.1);
			qlll.Transform(scaler);
			qurr.Transform(scaler);
			boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(qlll), Point3DOTB(qurr));

			// intersect roomshape with objects 
			BOPAlgo_Splitter aSplitter;
			TopTools_ListOfShape aLSObjects;
			aLSObjects.Append(sizedRoomShape);
			TopTools_ListOfShape aLSTools;

			TopExp_Explorer expl;

			std::vector<Value> qResult;
			std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> qProductList;
			qResult.clear();

			for (int j = 0; j < cSize; j++)
			{
				qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
				cluster->getHelper(j)->getIndexPointer()->query(bgi::intersects(qBox), std::back_inserter(qResult));

				if (qResult.size() == 0) { continue; }

				for (size_t k = 0; k < qResult.size(); k++)
				{

					LookupValue lookup = cluster->getHelper(j)->getLookup(qResult[k].second);
					IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);
					TopoDS_Shape shape;

					 if (std::get<3>(lookup))
					{
						shape = std::get<4>(lookup);
					}
					else {
						shape = cluster->getHelper(j)->getObjectShape(std::get<0>(lookup), true);
					}

					qProductList.emplace_back(std::make_tuple(qProduct, cluster->getHelper(j)->getObjectShape(std::get<0>(lookup), false)));

					int sCount = 0;

					for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next()) {
						aLSTools.Append(TopoDS::Solid(expl.Current()));
						sCount++;
					}

					if (sCount == 0)
					{
						if (qProduct->data().type()->name() == "IfcSlab") // TODO replace this statement
						{
							for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) {
								aLSTools.Append(TopoDS::Face(expl.Current()));
							}
						}

					}

					/*if (sCount == 0 && std::get<0>(lookup)->data().type()->name() == "IfcRoof")
					{
						std::cout << std::get<0>(lookup)->data().toString() << std::endl;

						BRep_Builder brepBuilder2;
						BRepBuilderAPI_Sewing brepSewer2;

						TopoDS_Solid newSolid;
						brepBuilder2.MakeSolid(newSolid);

						std::vector<TopoDS_Face> faceList;
						std::vector<TopoDS_Face> newFaceList;

						for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) {
							faceList.emplace_back(TopoDS::Face(expl.Current()));
						}

						for (size_t l = 0; l < faceList.size(); l++)
						{
							std::vector<gp_Pnt> facePointList;
							TopoDS_Wire wire;
							for (expl.Init(faceList[l], TopAbs_VERTEX); expl.More(); expl.Next())
							{
								TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
								facePointList.emplace_back(BRep_Tool::Pnt(vertex));
							}

							std::vector<TopoDS_Edge> faceEdgeList;
							for (size_t m = 0; m < facePointList.size(); m += 2)
							{
								BRepBuilderAPI_MakeWire(wire, BRepBuilderAPI_MakeEdge(facePointList[m], facePointList[m + 1]));
							}
							newFaceList.emplace_back(BRepBuilderAPI_MakeFace(wire));
						}

						for (size_t l = 0; l < newFaceList.size(); l++) { brepSewer2.Add(newFaceList[l]); }

						brepSewer2.Perform();
						brepBuilder2.Add(newSolid, brepSewer2.SewedShape());
						//aLSTools.Append(newSolid);
					}*/
					/*if (std::get<0>(lookup)->data().type()->name() == "IfcSlab" ||
						std::get<0>(lookup)->data().type()->name() == "IfcRoof" || 
						std::get<0>(lookup)->data().type()->name() == "IfcWall" || 
						std::get<0>(lookup)->data().type()->name() == "IfcWindow")
					{
						aLSTools.Append(shape);
					}
					else
					{
						for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next()) {
							aLSTools.Append(TopoDS::Solid(expl.Current()));

						}
					}*/
				}
			}


			aLSTools.Reverse();
			aSplitter.SetArguments(aLSObjects);
			aSplitter.SetTools(aLSTools);
			aSplitter.SetRunParallel(Standard_True);
			//aSplitter.SetFuzzyValue(0.001);
			aSplitter.SetNonDestructive(Standard_True);

			aSplitter.Perform();

			const TopoDS_Shape& aResult = aSplitter.Shape(); // result of the operation

			//WriteToSTEP(aResult, std::to_string(i));

			// get roomshape
			std::vector<TopoDS_Solid> solids;
			for (expl.Init(aResult, TopAbs_SOLID); expl.More(); expl.Next()) { solids.emplace_back(TopoDS::Solid(expl.Current())); }
			if (solids.size() <= 1) {
				continue;
			}

			// get roomshape
			int BiggestRoom = -1;

			std::vector<int> outSideIndx;
			outSideIndx.clear();

			// eleminate very low shapes (<2m)
			for (size_t j = 0; j < solids.size(); j++)
			{
				double shapeMaxZ = -9999;
				double shapeMinZ = 9999;

				for (expl.Init(solids[j], TopAbs_VERTEX); expl.More(); expl.Next()) {
					gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(expl.Current()));

					if (shapeMaxZ == -9999) { shapeMaxZ = p.Z(); }
					else if (shapeMaxZ < p.Z()) { shapeMaxZ = p.Z(); }

					if (shapeMinZ == 9999) { shapeMinZ = p.Z(); }
					else if (shapeMinZ > p.Z()) { shapeMinZ = p.Z(); }
				}

				if (shapeMaxZ - shapeMinZ < 1)
				{
					outSideIndx.emplace_back(j);
				}
			}
			std::sort(outSideIndx.begin(), outSideIndx.end(), std::greater<int>());
			for (size_t j = 0; j < outSideIndx.size(); j++) { solids.erase(solids.begin() + outSideIndx[j]); }
			outSideIndx.clear();

			if(hasInsidePoint)
			{
				for (size_t j = 0; j < solids.size(); j++)
				{
					BRepClass3d_SolidClassifier insideChecker;
					insideChecker.Load(solids[j]);
					insideChecker.Perform(inRoomPoint, 0.001);

					if (!insideChecker.State())
					{
						BiggestRoom = j;
						BRepGProp::VolumeProperties(solids[BiggestRoom], gprop);
						double volume = gprop.Mass();
						roomAreaList_.emplace_back(volume);
						break;
					}

				}
			}
			else {
				continue;
			}

			if (BiggestRoom == -1)
			{
				continue;
			}

			// Make a space object
			TopoDS_Shape UnitedScaledRoom = solids[BiggestRoom];
			if (unitScale != 1)
			{
				gp_Trsf UnitScaler;
				UnitScaler.SetScale({ 0.0, 0.0, 0.0 }, unitScale);
				UnitedScaledRoom = BRepBuilderAPI_Transform(solids[BiggestRoom], UnitScaler).ModifiedShape(solids[BiggestRoom]);
			}

			// find correct storey
			double lowX = 9999;
			double lowY = 9999;
			double lowZ = 9999;

			for (expl.Init(UnitedScaledRoom, TopAbs_VERTEX); expl.More(); expl.Next()) {
				gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(expl.Current()));
				if (p.X() < lowZ) { lowX = p.X(); }
				if (p.Y() < lowZ) { lowY = p.Y(); }
				if (p.Z() < lowZ) { lowZ = p.Z(); }
			}

			double elevDistance = 9999;
			int storeyIndx = 0;
			for (size_t j = 0; j < elevations.size(); j++)
			{
				double d = elevations[j] - lowZ;
				if (d < elevDistance && d >= 0)
				{
					storeyIndx = j;
					elevDistance = d;
				}
			}

			IfcSchema::IfcLocalPlacement* relativeLoc = hierarchyHelper.addLocalPlacement(0, lowX, lowY, lowZ);
			auto t = cluster->getHelper(roomLoc)->getSourceFile()->addEntity(relativeLoc)->as<IfcSchema::IfcLocalPlacement>();

			TopoDS_Shape unMovedUnitedScaledRoom = UnitedScaledRoom;
			gp_Trsf relativeMovement;
			relativeMovement.SetTranslation({ 0,0,0 }, { -lowX, -lowY, -lowZ });
			UnitedScaledRoom.Move(relativeMovement);

			IfcSchema::IfcProductRepresentation* roomRep = IfcGeom::serialise(STRINGIFY(IfcSchema), UnitedScaledRoom, false)->as<IfcSchema::IfcProductRepresentation>();
			if (roomRep == 0)
			{
				std::cout << "wa" << std::endl;
				continue;
			}
			// find semantic data
			std::vector < IfcSchema::IfcSpace* > semanticDataList;
			for (size_t j = 0; j < cSize; j++)
			{
				BRepClass3d_SolidClassifier insideChecker;
				std::vector<gp_Pnt> centerPoints = cluster->getHelper(j)->getRoomCenters();
				insideChecker.Load(unMovedUnitedScaledRoom);

				for (size_t k = 0; k < centerPoints.size(); k++)
				{
					insideChecker.Perform(centerPoints[k], 0.01);

					if (!insideChecker.State())
					{
						semanticDataList.emplace_back(std::get<0>(cluster->getHelper(j)->getRLookup(k)));
					}
				}
				break;
			}

			std::vector<std::string> semanticData = getSemanticMatch(semanticDataList, roomnum);

#ifdef USE_IFC4
			IfcSchema::IfcSpace* room = new IfcSchema::IfcSpace(
				IfcParse::IfcGlobalId(),														// GlobalId
				0,																				// OwnerHistory
				semanticData[0],																// Name
				semanticData[2],																// Description
				boost::none,																	// Object type
				t,																				// Object Placement	
				roomRep,																		// Representation
				semanticData[1],																// Long name
				IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT,			// Composition Type	
				boost::none,																	// Predefined Type
				boost::none																		// Elevation with Flooring
			);
#else
			IfcSchema::IfcSpace* room = new IfcSchema::IfcSpace(
				IfcParse::IfcGlobalId(),														// GlobalId
				0,																				// OwnerHistory
				semanticData[0],																// Name
				semanticData[2],																// Description
				boost::none,																	// Object type
				t,																				// Object Placement
				roomRep,																		// Representation
				semanticData[1],																// Long name
				IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT,			// Composition Type	
				IfcSchema::IfcInternalOrExternalEnum::IfcInternalOrExternal_INTERNAL,			// Interior or exterior space
				boost::none																		// Elevation with Flooring
			);
#endif // USE_IFC4

			// get rel space boundaries
			BRepExtrema_DistShapeShape distanceMeasurer;
			distanceMeasurer.LoadS1(unMovedUnitedScaledRoom);
			
			std::vector<IfcSchema::IfcRelSpaceBoundary*> boundaryList;
			std::vector<IfcSchema::IfcDoor*> connectedDoors; // TODO improve door implementation

			for (size_t j = 0; j < qProductList.size(); j++)
			{
				bool isConnected = true;

				// find objects that are close
				IfcSchema::IfcProduct* qProduct = std::get<0>(qProductList[j]);
				TopoDS_Shape qShape = std::get<1>(qProductList[j]);

				distanceMeasurer.LoadS2(qShape);

				distanceMeasurer.Perform();

				double distance = distanceMeasurer.Value();

				if (distance > 0.5)
				{
					isConnected = false;
				}
				else if (distance < 0.0001) // if distance is 0 it is known it is connected
				{
					isConnected = true;


				}
				else if (distance < 0.5)
				{
					int counter = 0;

					double x1 = 0;
					double x2 = 0;
					double y1 = 0;
					double y2 = 0;
					double z1 = 0;
					double z2 = 0;

					for (size_t k = 1; k < distanceMeasurer.NbSolution(); k++)
					{
						gp_Pnt p1 = distanceMeasurer.PointOnShape1(k);
						gp_Pnt p2 = distanceMeasurer.PointOnShape2(k);

						x1 += p1.X();
						x2 += p2.X();
						y1 += p1.Y();
						y2 += p2.Y();
						z1 += p1.Z();
						z2 += p2.Z();

						counter++;
					}

					gp_Pnt p1n(x1 / counter, y1 / counter, z1 / counter);
					gp_Pnt p2n(x2 / counter, y2 / counter, z2 / counter);
					
					if (isnan(p1n.X()) || isnan(p1n.Y()) || isnan(p1n.Z()) ||
						isnan(p2n.X()) || isnan(p2n.Y()) || isnan(p2n.Z()))
					{
						isConnected = false;
					}
					else
					{
						TopoDS_Edge shortestEdge = BRepBuilderAPI_MakeEdge(p1n, p2n);

						for (size_t k = 0; k < qProductList.size(); k++)
						{
							if (j == k) { continue; }

							IfcSchema::IfcProduct* matchingProduct = std::get<0>(qProductList[k]);

							IntTools_EdgeFace intersector;
							intersector.SetEdge(shortestEdge);

							for (expl.Init(std::get<1>(qProductList[k]), TopAbs_FACE); expl.More(); expl.Next()) {
								intersector.SetFace(TopoDS::Face(expl.Current()));
								intersector.Perform();

								if (intersector.CommonParts().Size() > 0) {
									isConnected = false;
									break;
								}
							}
							if (!isConnected)
							{
								break;
							}
						}
					}
				}
				if (isConnected)
				{
					IfcSchema::IfcRelSpaceBoundary* boundary = new IfcSchema::IfcRelSpaceBoundary(
						IfcParse::IfcGlobalId(),
						0,
						std::string(""),
						std::string(""),
						room,
						qProduct->as<IfcSchema::IfcElement>(),
						0,
						IfcSchema::IfcPhysicalOrVirtualEnum::IfcPhysicalOrVirtual_PHYSICAL,
						IfcSchema::IfcInternalOrExternalEnum::IfcInternalOrExternal_INTERNAL
					);

					boundaryList.emplace_back(boundary);
				}
			}

			cluster->getHelper(roomLoc)->getSourceFile()->addEntity(room);
			
			for (size_t j = 0; j < boundaryList.size(); j++)
			{
				cluster->getHelper(roomLoc)->getSourceFile()->addEntity(boundaryList[j]);
			}

			roomProducts.get()->push(room);

			roomObject* rObject = new roomObject(room, roomObjectList_.size());
			roomObjectList_.emplace_back(rObject);
			updateConnections(unMovedUnitedScaledRoom, rObject, roomObjectList_, qBox, cluster);
			roomnum++;


		}
	}


	outputFieldToFile();

	std::cout << std::endl;
	std::cout << std::endl;

	// go through all old space objects and remove element space 
	// TODO keep spaces not recreated?
	for (size_t i = 0; i < cSize; i++)
	{
		std::vector<roomLookupValue> roomValues = cluster->getHelper(i)->getFullRLookup();
		for (size_t j = 0; j < roomValues.size(); j++)
		{
			IfcSchema::IfcSpace* space = std::get<0>(roomValues[j]);
			if (space->CompositionType() == IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT)
			{
				cluster->getHelper(i)->getSourceFile()->removeEntity(space);
			}
		}
	}

	if (roomObjectList_.size() == 0) { return;}

	floorProcessor::sortObjects(cluster->getHelper(roomLoc), roomProducts);

	// apply connectivity data to the rooms
	for (size_t i = 0; i < roomObjectList_.size(); i++)
	{
		roomObject* rObject = roomObjectList_[i];
		auto connections = rObject->getConnections();
		std::string description = "";
		for (size_t i = 0; i <connections.size(); i++) { description = description + connections[i]->getSelf()->Name() + ", "; }
		rObject->getSelf()->setDescription(rObject->getSelf()->Description() + description);


	}

	createGraph(cluster);

}

void voxelfield::updateConnections(TopoDS_Shape room, roomObject* rObject, std::vector<roomObject*> rObjectList, boost::geometry::model::box<BoostPoint3D> qBox, helperCluster* cluster)
{
	int cSize = cluster->getSize();

	int doorCount = 0;
	int stairCount = 0;

	// find connectivity data
	// Get lowest Face
	std::vector<TopoDS_Face> roomFootPrintList = getRoomFootprint(room);

	TopoDS_Vertex lDoorP;

	std::vector<Value> qResult;
	TopExp_Explorer expl;

	for (int j = 0; j < cSize; j++)
	{
		qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
		cluster->getHelper(j)->getConnectivityIndexPointer()->query(bgi::intersects(qBox), std::back_inserter(qResult));

		for (size_t k = 0; k < qResult.size(); k++)
		{

			ConnectLookupValue lookup = cluster->getHelper(j)->getCLookup(qResult[k].second);
			TopoDS_Shape objectShape = cluster->getHelper(j)->getObjectShape(std::get<0>(lookup));

			if (std::get<0>(lookup)->data().type()->name() == "IfcDoor")
			{
				bool found = false;


				gp_Pnt groundObjectPoint = std::get<1>(lookup)[0];

				for (size_t l = 0; l < roomFootPrintList.size(); l++)
				{
					for (expl.Init(roomFootPrintList[l], TopAbs_EDGE); expl.More(); expl.Next())
					{
						TopoDS_Edge edge = TopoDS::Edge(expl.Current());
						TopExp_Explorer edgeExpl;
						std::vector<gp_Pnt> edgePoints;
						edgePoints.clear();

						for (edgeExpl.Init(edge, TopAbs_VERTEX); edgeExpl.More(); edgeExpl.Next()) {
							TopoDS_Vertex vertex = TopoDS::Vertex(edgeExpl.Current());
							edgePoints.emplace_back(BRep_Tool::Pnt(vertex));
						}

						if (edgePoints.size() != 2) { std::cout << "error" << std::endl; }

						double baseDistance = edgePoints[0].Distance(edgePoints[1]);
						double triDistance = edgePoints[0].Distance(groundObjectPoint) + edgePoints[1].Distance(groundObjectPoint);

						double buffer = 0.3;

						if (abs(baseDistance - triDistance) <= buffer)
						{
							doorCount++;
							std::get<2>(lookup)->emplace_back(rObject);

							if (std::get<2>(lookup)->size() == 2)
							{
								std::get<2>(lookup)[0][0]->addConnection(std::get<2>(lookup)[0][1]);
								std::get<2>(lookup)[0][1]->addConnection(std::get<2>(lookup)[0][0]);
							}

							found = true;
						}
						if (found)
						{
							break;
						}
					}
				}

				
			}
			if (std::get<0>(lookup)->data().type()->name() == "IfcStair") {

				gp_Pnt groundObjectPoint = std::get<1>(lookup)[0];

				gp_Trsf stairOffset;
				stairOffset.SetTranslation({ 0,0,0 }, { 0,0,0.5 });

				gp_Pnt topObjectPoint = std::get<1>(lookup)[1];

				BRepClass3d_SolidClassifier insideChecker;
				insideChecker.Load(room);
				insideChecker.Perform(groundObjectPoint.Transformed(stairOffset), 0.01);

				if (!insideChecker.State())
				{
					std::get<2>(lookup)->emplace_back(rObject);
					stairCount++;
					if (std::get<2>(lookup)->size() == 2)
					{
						std::get<2>(lookup)[0][0]->addConnection(std::get<2>(lookup)[0][1]);
						std::get<2>(lookup)[0][1]->addConnection(std::get<2>(lookup)[0][0]);
					}
					continue;
				}
				insideChecker.Perform(topObjectPoint, 0.01);

				if (!insideChecker.State())
				{
					std::get<2>(lookup)->emplace_back(rObject);
					stairCount++;
					if (std::get<2>(lookup)->size() == 2)
					{
						std::get<2>(lookup)[0][0]->addConnection(std::get<2>(lookup)[0][1]);
						std::get<2>(lookup)[0][1]->addConnection(std::get<2>(lookup)[0][0]);
					}
					continue;
				}

			}
		}
	}
	 rObject->getSelf()->setDescription("Has " + std::to_string(doorCount) + " unique doors and " + std::to_string(stairCount) + " unique stairs. Connected to : ");
}
	
std::vector<int> voxelfield::growRoom(int startIndx, int roomnum)
{
	std::vector<int> buffer = { startIndx };
	std::vector<int> totalRoom = { startIndx };

	Assignment_[startIndx] = 1;

	bool isOutSide = false;

	while (buffer.size() > 0)
	{
		std::vector<int> tempBuffer;
		for (size_t j = 0; j < buffer.size(); j++)
		{
			int currentIdx = buffer[j];
			voxel* currentBoxel = VoxelLookup_[currentIdx];
			currentBoxel->getCenterPoint();
			currentBoxel->addRoomNumber(roomnum);

			if (Assignment_[currentIdx] == -1)
			{
				continue;
			}

			// find neighbours
			std::vector<int> neighbourIndx = getNeighbours(currentIdx);

			if (neighbourIndx.size() < 26) { isOutSide = true; }

			for (size_t k = 0; k < neighbourIndx.size(); k++)
			{
				// exlude if already assigned
				if (Assignment_[neighbourIndx[k]] == 0) {
					bool dupli = false;
					for (size_t l = 0; l < tempBuffer.size(); l++)
					{
						// exlude if already in buffer
						if (neighbourIndx[k] == tempBuffer[l])
						{
							dupli = true;
							break;
						}
					}
					if (!dupli)
					{
						tempBuffer.emplace_back(neighbourIndx[k]);
						totalRoom.emplace_back(neighbourIndx[k]);
						Assignment_[neighbourIndx[k]] = 1;
					}
				}
				else if (Assignment_[neighbourIndx[k]] == -1) {
					bool dupli = false;

					for (size_t l = 0; l < totalRoom.size(); l++)
					{
						if (neighbourIndx[k] == totalRoom[l]) {
							dupli = true;
						}
					}
					if (!dupli)
					{
						totalRoom.emplace_back(neighbourIndx[k]);
						tempBuffer.emplace_back(neighbourIndx[k]);
					}
				}
			}
		}
		buffer.clear();
		buffer = tempBuffer;
	}
	if (isOutSide)
	{
		for (size_t k = 0; k < totalRoom.size(); k++)
		{
			int currentIdx = totalRoom[k];
			voxel* currentBoxel = VoxelLookup_[currentIdx];
			if (!currentBoxel->getIsIntersecting())
			{
				currentBoxel->setOutside();
			}
		}
		return{};
	}
	return totalRoom;
}

voxel::voxel(BoostPoint3D center, double sizeXY, double sizeZ)
{
	sizeXY_ = sizeXY;
	sizeZ_ = sizeZ;
	center_ = center;

	gp_Pnt minPoint(bg::get<0>(center) - 1 / 2 * sizeXY, bg::get<1>(center) - 1 / 2 * sizeXY, bg::get<2>(center) - 1 / 2 * sizeZ);
	gp_Pnt maxPoint(bg::get<0>(center) + 1 / 2 * sizeXY, bg::get<1>(center) + 1 / 2 * sizeXY, bg::get<2>(center) + 1 / 2 * sizeZ);
}

bg::model::box<BoostPoint3D> voxel::getVoxelGeo()
{
	double offsetXY = sizeXY_ / 2;
	double offsetZ = sizeZ_ / 2;

	BoostPoint3D lll (bg::get<0>(center_) - offsetXY, bg::get<1>(center_) - offsetXY, bg::get<2>(center_) - offsetZ);
	BoostPoint3D urr(bg::get<0>(center_) + offsetXY, bg::get<1>(center_) + offsetXY, bg::get<2>(center_) + offsetZ);
	
	return bg::model::box<BoostPoint3D>(lll, urr);
}

std::vector<gp_Pnt> voxel::getCornerPoints(double angle)
{
	auto boxelGeo = getVoxelGeo();

	auto minPoint = Point3DBTO(boxelGeo.min_corner());
	auto maxPoint = Point3DBTO(boxelGeo.max_corner());

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

	for (size_t i = 0; i < pointList.size(); i++)
	{
		pointList[i] = rotatePointWorld(pointList[i], -angle);
	}


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

std::vector<std::vector<int>> voxel::getVoxelEdges()
{
	return {
		{ 0, 1},
		{ 1, 2},
		{ 2, 3},
		{ 3, 0},
		{ 4, 5},
		{ 5, 6},
		{ 6, 7},
		{ 7, 4},
		{ 1, 5},
		{ 2, 4},
		{ 3, 7},
		{ 0, 6}
	};
}

double voxel::tVolume(gp_Pnt p, const std::vector<gp_Pnt> vertices) {
	BoostPoint3D p1(vertices[0].X() - vertices[1].X(), vertices[0].Y() - vertices[1].Y(), vertices[0].Z() - vertices[1].Z());
	BoostPoint3D p2(vertices[1].X() - p.X(), vertices[1].Y() - p.Y(), vertices[1].Z() - p.Z());
	BoostPoint3D p3(vertices[2].X() - p.X(), vertices[2].Y() - p.Y(), vertices[2].Z() - p.Z());

	return bg::dot_product(p1, bg::cross_product(p2, p3))/6;
}

bool voxel::checkIntersecting(LookupValue lookup, std::vector<gp_Pnt> voxelPoints, helper* h)
{
	std::vector<std::vector<int>> vets = getVoxelEdges();

	IfcSchema::IfcProduct* product = std::get<0>(lookup);
	std::vector<gp_Pnt> productPoints = h->getObjectPoints(product, false, true);
	
	// check if any cornerpoints fall inside voxel
	if (linearEqIntersection(productPoints, voxelPoints))
	{
		isIntersecting_ = true;
		return true;
	}

	std::vector<std::vector<int>> triangleVoxels = getVoxelTriangles();
	
	for (size_t i = 0; i < triangleVoxels.size(); i++)
	{
		std::vector<gp_Pnt> voxelTriangle = { voxelPoints[triangleVoxels[i][0]], voxelPoints[triangleVoxels[i][1]], voxelPoints[triangleVoxels[i][2]] };

		for (size_t k = 0; k < productPoints.size(); k+=2)
		{
			if (checkIntersecting({ productPoints[k], productPoints[k + 1] }, voxelTriangle))
			{
				isIntersecting_ = true;
				return true;
			}
		}
	}

	// check with triangulated object
	std::vector<std::vector<gp_Pnt>> triangleMesh = std::get<1>(lookup);

	for (size_t i = 0; i < triangleMesh.size(); i++)
	{
		std::vector<gp_Pnt> triangle = triangleMesh[i];

		for (size_t k = 0; k < vets.size(); k++)
		{
			if (checkIntersecting({ voxelPoints[vets[k][0]], voxelPoints[vets[k][1]] }, triangle))
			{
				isIntersecting_ = true;
				return true; 
			}
		}
	}
	return false;
}

bool voxel::linearEqIntersection(std::vector<gp_Pnt> productPoints, std::vector<gp_Pnt> voxelPoints)
{
	// check if any cornerpoints fall inside voxel
	gp_Pnt p1 = voxelPoints[0];
	gp_Pnt p2 = voxelPoints[1];
	gp_Pnt p3 = voxelPoints[3];

	if (p2.Y() - p1.Y() == 0)
	{
		for (size_t i = 0; i < productPoints.size(); i++)
		{
			gp_Pnt currentPP = productPoints[i];

			if (currentPP.Z() < p1.Z() || currentPP.Z() > voxelPoints[4].Z()) { continue; }
			if (currentPP.X() < p1.X() && currentPP.X() < voxelPoints[4].X() ||
				currentPP.X() > p1.X() && currentPP.X() > voxelPoints[4].X()) {
				continue;
			}
			if (currentPP.Y() < p1.Y() && currentPP.Y() < voxelPoints[4].Y() ||
				currentPP.Y() > p1.Y() && currentPP.Y() > voxelPoints[4].Y()) {
				continue;
			}
			return true;
		}
		return false;
	}

	double a1 = (p2.Y() - p1.Y()) / (p2.X() - p1.X());
	double b11 = p2.Y() - a1 * p2.X();
	double b12 = p3.Y() - a1 * p3.X();

	double a2 = -1 / a1;
	double b21 = p3.Y() - a2 * p3.X();
	double b22 = p2.Y() - a2 * p2.X();

	for (size_t i = 0; i < productPoints.size(); i++)
	{
		gp_Pnt currentPP = productPoints[i];

		// check if point is in z range
		if (currentPP.Z() < p1.Z() || currentPP.Z() > voxelPoints[4].Z()) { continue; }

		// check if point is in voxel
		double x = currentPP.X();

		double y11 = a1 * x + b11;
		double y12 = a1 * x + b12;

		if (currentPP.Y() < y11 && currentPP.Y() < y12 ||
			currentPP.Y() > y11 && currentPP.Y() > y12) {
			continue;
		}

		double y21 = a2 * x + b21;
		double y22 = a2 * x + b22;

		if (currentPP.Y() < y21 && currentPP.Y() > y22 ||
			currentPP.Y() > y21 && currentPP.Y() < y22)
		{
			return true;
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

	if (leftFinal > 0 && rightFinal < 0 || rightFinal > 0 && leftFinal < 0) { return true; }
	return false;
}
