#include "roomProcessor.h"

void WriteToSTEP(TopoDS_Solid shape, std::string addition) {
	std::string path = "D:/Documents/Uni/Thesis/sources/Models/exports/step" + addition + ".stp";
	
	STEPControl_Writer writer;

	writer.Transfer(shape, STEPControl_ManifoldSolidBrep);
	IFSelect_ReturnStatus stat = writer.Write(path.c_str());

	//std::cout << "stat: " << stat << std::endl;
}

void WriteToSTEP(TopoDS_Shape shape, std::string addition) {
	std::string path = "D:/Documents/Uni/Thesis/sources/Models/exports/step" + addition + ".stp";
	
	STEPControl_Writer writer;

	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next()) {
		writer.Transfer(expl.Current(), STEPControl_ManifoldSolidBrep);
	}

	IFSelect_ReturnStatus stat = writer.Write(path.c_str());

	//std::cout << "stat: " << stat << std::endl;
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

gp_Pnt voxelfield::getLowestPoint(TopoDS_Shape shape)
{
	double lowestZ = 9999;
	gp_Pnt lowestP(0,0,0);


	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() < lowestZ)
		{
			lowestZ = p.Z();
			lowestP = p;
		}

		double sumX = 0;
		double sumY = 0;
		int aP = 0;

		for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			if (p.Z() == lowestZ)
			{
				aP++;
				sumX += p.X();
				sumY += p.Y();
			}
		}

		return gp_Pnt(sumX / aP, sumY / aP, lowestZ);
	}
	return lowestP;
}


gp_Pnt voxelfield::getHighestPoint(TopoDS_Shape shape)
{
	double highestZ = -9999;
	gp_Pnt highestP(0, 0, 0);

	TopExp_Explorer expl;

	for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() > highestZ)
		{
			highestZ = p.Z();
			highestP = p;
		}
	}

	double sumX = 0;
	double sumY = 0;
	int aP = 0;

	for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() == highestZ)
		{
			aP++;
			sumX += p.X();
			sumY += p.Y();
		}
	}

	return gp_Pnt(sumX/aP, sumY/aP, highestZ);
}


void voxelfield::addVoxel(int indx, helperCluster* cluster)
{
	int cSize = cluster->getSize();
	auto midPoint = relPointToWorld(linearToRelative<BoostPoint3D>(indx));
	voxel* boxel = new voxel(midPoint, voxelSize_);

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
	for (int i = 0; i < totalVoxels_; i++) { addVoxel(i, cluster); }
	std::cout << "Room Growing" << std::endl;
	// asign rooms
	int roomnum = 0;
	int temps = 0;
	std::cout << "room detection" << std::endl;

	std::vector<roomObject*> roomObjects; // start graph data

	for (int i = 0; i < totalVoxels_; i++)
	{
		if (Assignment_[i] == 0) // Find unassigned voxel
		{
			std::vector<int> totalRoom = growRoom(i, roomnum);
			if (totalRoom.size() == 0) { continue; }

			std::cout << "new room nr: " << roomnum + 1 << std::endl;

			BRep_Builder brepBuilder;
			BRepBuilderAPI_Sewing brepSewer;

			TopoDS_Shell shell;
			brepBuilder.MakeShell(shell);
			TopoDS_Solid roughRoomShape;
			brepBuilder.MakeSolid(roughRoomShape);

			// create bbox around rough room shape
			gp_Pnt lll(9999, 9999, 9999);
			gp_Pnt urr(-9999, -9999, -9999);

			for (size_t j = 1; j < totalRoom.size(); j++)
			{
				voxel* currentBoxel = VoxelLookup_[totalRoom[j]];
				std::vector<gp_Pnt> cornerPoints = currentBoxel->getCornerPoints(planeRotation_);
				for (size_t k = 0; k < cornerPoints.size(); k++)
				{
					auto currentCorner = rotatePointWorld(cornerPoints[k], planeRotation_);
					if (urr.X() < currentCorner.X()) { urr.SetX(currentCorner.X()); }
					if (urr.Y() < currentCorner.Y()) { urr.SetY(currentCorner.Y()); }
					if (urr.Z() < currentCorner.Z()) { urr.SetZ(currentCorner.Z()); }
					if (lll.X() > currentCorner.X()) { lll.SetX(currentCorner.X()); }
					if (lll.Y() > currentCorner.Y()) { lll.SetY(currentCorner.Y()); }
					if (lll.Z() > currentCorner.Z()) { lll.SetZ(currentCorner.Z()); }
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
			scaler.SetScale(pC, 1.2);
			TopoDS_Shape sizedRoomShape = BRepBuilderAPI_Transform(roughRoomShape, scaler).ModifiedShape(roughRoomShape);
			p0.Transform(scaler);
			p4.Transform(scaler);

			// intersect roomshape with walls 
			BOPAlgo_Splitter aSplitter;

			TopTools_ListOfShape aLSObjects;
			aLSObjects.Append(sizedRoomShape);
			TopTools_ListOfShape aLSTools;

			TopExp_Explorer expl;

			//TODO do intersection process
			boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(p0), Point3DOTB(p4));

			std::vector<Value> qResult;
			qResult.clear();

			for (int j = 0; j < cSize; j++)
			{
				qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
				cluster->getHelper(j)->getIndexPointer()->query(bgi::intersects(qBox), std::back_inserter(qResult));

				if (qResult.size() == 0) { continue; }

				for (size_t k = 0; k < qResult.size(); k++)
				{
					LookupValue lookup = cluster->getHelper(j)->getLookup(qResult[k].second);
					TopoDS_Shape shape = cluster->getHelper(j)->getObjectShape(std::get<0>(lookup));

					if (std::get<0>(lookup)->data().type()->name() == "IfcSlab")
					{
						aLSTools.Append(shape);
					}
					else
					{
						for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next()) {
							aLSTools.Append(TopoDS::Solid(expl.Current()));
						}
					}
				}
			}

			aLSTools.Reverse();

			aSplitter.SetArguments(aLSObjects);
			aSplitter.SetTools(aLSTools);

			aSplitter.SetRunParallel(Standard_True);
			//aSplitter.SetFuzzyValue(0.0001);
			aSplitter.SetNonDestructive(Standard_True);

			aSplitter.Perform();

			const TopoDS_Shape& aResult = aSplitter.Shape(); // result of the operation

			// get roomshape
			std::vector<TopoDS_Solid> solids;
			for (expl.Init(aResult, TopAbs_SOLID); expl.More(); expl.Next()) { solids.emplace_back(TopoDS::Solid(expl.Current())); }

			if (solids.size() == 1) {
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

			// eliminate outside shape by Z values
			double maxZ = -9999;
			double minZ = 9999;

			for (expl.Init(sizedRoomShape, TopAbs_VERTEX); expl.More(); expl.Next()) {
				gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(expl.Current()));

				if (maxZ == -9999) { maxZ = p.Z(); }
				else if (maxZ < p.Z()) { maxZ = p.Z(); }

				if (minZ == 9999) { minZ = p.Z(); }
				else if (minZ > p.Z()) { minZ = p.Z(); }

			}

			for (size_t j = 0; j < solids.size(); j++)
			{
				for (expl.Init(solids[j], TopAbs_VERTEX); expl.More(); expl.Next()) {
					gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(expl.Current()));

					if (p.Z() >= maxZ || p.Z() <= minZ)
					{
						outSideIndx.emplace_back(j);
						break;
					}
				}
			}

			std::sort(outSideIndx.begin(), outSideIndx.end(), std::greater<int>());
			for (size_t j = 0; j < outSideIndx.size(); j++) { solids.erase(solids.begin() + outSideIndx[j]); }

			if (solids.size() < 1) { continue; }
			else if (solids.size() == 1)
			{
				BiggestRoom = 0;
			}
			else {

				std::vector<gp_Pnt> roomShapePoints;
				roomShapePoints.clear();

				// filter for volume 
				double maxVolume = 0;

				for (size_t j = 0; j < solids.size(); j++)
				{
					BRepGProp::VolumeProperties(solids[j], gprop);
					double volume = gprop.Mass();
					if (maxVolume < volume)
					{
						maxVolume = volume;
						BiggestRoom = j;
					}
				}
			}
			//std::cout << "in" << std::endl;

			// Make a space object
			//std::cout << "b: " << solids.size() << std::endl;
			TopoDS_Shape UnitedScaledRoom = solids[BiggestRoom];
			//std::cout << "out" << std::endl;
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
			std::vector < std::tuple<IfcSchema::IfcSpace*, gp_Pnt>> semanticDataList;
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
						semanticDataList.emplace_back(std::make_tuple(std::get<0>(cluster->getHelper(j)->getRLookup(k)), centerPoints[k]));
					}
				}
			}

			// unload semantic data
			IfcSchema::IfcSpace* matchingSpaceObject = std::get<0>(semanticDataList[0]);
			std::string semanticName = "Automatic Space";
			std::string semanticLongName = "Automatic Space: " + std::to_string(roomnum);
			std::string semanticDescription = "";

			if (matchingSpaceObject->hasName()) { semanticName = matchingSpaceObject->Name(); }
			if (matchingSpaceObject->hasLongName()) { semanticLongName = matchingSpaceObject->LongName(); }
			if (matchingSpaceObject->hasDescription()) { semanticDescription = matchingSpaceObject->Description(); }

#ifdef USE_IFC4
			IfcSchema::IfcSpace* room = new IfcSchema::IfcSpace(
				IfcParse::IfcGlobalId(),														// GlobalId
				0,																				// OwnerHistory
				semanticName,																	// Name
				semanticDescription,																// Description
				boost::none,																	// Object type
				t,																				// Object Placement	
				roomRep,																		// Representation
				semanticLongName,															// Long name
				IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT,			// Composition Type	
				boost::none,																	// Predefined Type
				boost::none																		// Elevation with Flooring
			);
#else
			IfcSchema::IfcSpace* room = new IfcSchema::IfcSpace(
				IfcParse::IfcGlobalId(),														// GlobalId
				0,																				// OwnerHistory
				std::string("Space"),															// Name
				std::string(std::to_string(roomnum)),											// Description
				boost::none,																	// Object type
				t,																				// Object Placement
				roomRep,																		// Representation
				std::string("Spaaaace"),														// Long name
				IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT,			// Composition Type	
				IfcSchema::IfcInternalOrExternalEnum::IfcInternalOrExternal_INTERNAL,			// Interior or exterior space
				boost::none																		// Elevation with Flooring
			);
#endif // USE_IFC4
			cluster->getHelper(roomLoc)->getSourceFile()->addEntity(room);
			roomProducts.get()->push(room);

			roomObject* rObject = new roomObject(room, roomObjects.size());

			roomObjects.emplace_back(rObject);
			updateConnections(unMovedUnitedScaledRoom, rObject, roomObjects, qBox, cluster);
			roomnum++;
		}
	}

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

	outputFieldToFile();
	floorProcessor::sortObjects(cluster->getHelper(roomLoc), roomProducts);

	// apply connectivity data to the rooms
	for (size_t i = 0; i < roomObjects.size(); i++)
	{
		roomObject* rObject = roomObjects[i];
		auto connections = rObject->getConnections();

		std::string description = "";

		for (size_t i = 0; i <connections.size(); i++)
		{
			description = description + connections[i]->getSelf()->Name() + ", ";
		}

		rObject->getSelf()->setDescription(rObject->getSelf()->Description() + description);
	}

}

void voxelfield::updateConnections(TopoDS_Shape room, roomObject* rObject, std::vector<roomObject*> rObjectList, boost::geometry::model::box<BoostPoint3D> qBox, helperCluster* cluster)
{
	int cSize = cluster->getSize();

	int doorCount = 0;
	int stairCount = 0;

	// find connectivity data
	// Get lowest Face
	TopoDS_Face roomfootprint = getLowestFace(room);
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

			gp_Pnt groundObjectPoint = getLowestPoint(objectShape);

			if (std::get<0>(lookup)->data().type()->name() == "IfcDoor")
			{
				for (expl.Init(roomfootprint, TopAbs_EDGE); expl.More(); expl.Next())
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
						std::get<1>(lookup)->emplace_back(rObject);

						if (std::get<1>(lookup)->size() == 2)
						{
							std::get<1>(lookup)[0][0]->addConnection(std::get<1>(lookup)[0][1]);
							std::get<1>(lookup)[0][1]->addConnection(std::get<1>(lookup)[0][0]);
						}

						break;
					}
				}
			}
			else {
				gp_Trsf stairOffset;
				stairOffset.SetTranslation({ 0,0,0 }, { 0,0,0.5 });

				gp_Pnt topObjectPoint = getHighestPoint(objectShape);

				BRepClass3d_SolidClassifier insideChecker;
				insideChecker.Load(room);
				insideChecker.Perform(groundObjectPoint.Transformed(stairOffset), 0.01);

				if (!insideChecker.State())
				{
					std::get<1>(lookup)->emplace_back(rObject);
					stairCount++;
					if (std::get<1>(lookup)->size() == 2)
					{
						std::get<1>(lookup)[0][0]->addConnection(std::get<1>(lookup)[0][1]);
						std::get<1>(lookup)[0][1]->addConnection(std::get<1>(lookup)[0][0]);
					}
					continue;
				}
				insideChecker.Perform(topObjectPoint, 0.01);

				if (!insideChecker.State())
				{
					std::get<1>(lookup)->emplace_back(rObject);
					stairCount++;
					if (std::get<1>(lookup)->size() == 2)
					{
						std::get<1>(lookup)[0][0]->addConnection(std::get<1>(lookup)[0][1]);
						std::get<1>(lookup)[0][1]->addConnection(std::get<1>(lookup)[0][0]);
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
	std::vector<gp_Pnt> productPoints = h->getObjectPoints(product);
	
	// check if any cornerpoints fall inside voxel
	if (linearEqIntersection(productPoints, voxelPoints))
	{
		isIntersecting_ = true;
		return true;
	}

	// check with triangulated voxel
	std::vector<gp_Pnt> productPointsEdges = h->getObjectPoints(product, true);

	std::vector<std::vector<int>> triangleVoxels = getVoxelTriangles();
	
	for (size_t i = 0; i < triangleVoxels.size(); i++)
	{
		std::vector<gp_Pnt> voxelTriangle = { voxelPoints[triangleVoxels[i][0]], voxelPoints[triangleVoxels[i][1]], voxelPoints[triangleVoxels[i][2]] };

		for (size_t k = 0; k < productPointsEdges.size(); k+=2)
		{
			if (checkIntersecting({ productPointsEdges[k], productPointsEdges[k + 1] }, voxelTriangle))
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
