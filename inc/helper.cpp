#include "helper.h"


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

	/*TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next()) {
		writer.Transfer(expl.Current(), STEPControl_ManifoldSolidBrep);
	}

	IFSelect_ReturnStatus stat = writer.Write(path.c_str());*/

	writer.Transfer(shape, STEPControl_ManifoldSolidBrep);
	IFSelect_ReturnStatus stat = writer.Write(path.c_str());

	//std::cout << "stat: " << stat << std::endl;
}

void printPoint(gp_Pnt p) {
	std::cout << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << std::endl;
}

void printPoint(BoostPoint3D p) {
	std::cout << bg::get<0>(p) << ", " << bg::get<1>(p) << ", " << bg::get<2>(p) << ", " << std::endl;
}

void printFaces(TopoDS_Shape shape)
{
	//std::cout << "Shape:" << std::endl;
	std::vector<TopoDS_Face> faceList;

	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next())
	{
		faceList.emplace_back(TopoDS::Face(expl.Current()));
	}

	for (size_t i = 0; i < faceList.size(); i++)
	{
		std::cout << "Start of Face" << std::endl;
		for (expl.Init(faceList[i], TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			printPoint(p);
		}
		std::cout << "End of Face" << std::endl;
		std::cout << std::endl;
	}
	//std::cout << std::endl;
}


gp_Pnt rotatePointWorld(gp_Pnt p, double angle) {
	double pX = p.X();
	double pY = p.Y();
	double pZ = p.Z();

	return gp_Pnt(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}

BoostPoint3D rotatePointWorld(BoostPoint3D p, double angle) {
	double pX = bg::get<0>(p);
	double pY = bg::get<1>(p);
	double pZ = bg::get<2>(p);

	return BoostPoint3D(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}

std::tuple<gp_Pnt, gp_Pnt, double> rotatedBBoxDiagonal(std::vector<gp_Pnt> pointList, double angle) {
	
	bool isPSet = false;
	gp_Pnt lllPoint;
	gp_Pnt urrPoint;

	for (size_t i = 0; i < pointList.size(); i++)
	{
		gp_Pnt point = rotatePointWorld(pointList[i], angle);

		if (!isPSet)
		{
			isPSet = true;

			lllPoint = point;
			urrPoint = point;
		}
		else
		{
			if (point.X() < lllPoint.X()) { lllPoint.SetX(point.X()); }
			if (point.Y() < lllPoint.Y()) { lllPoint.SetY(point.Y()); }
			if (point.Z() < lllPoint.Z()) { lllPoint.SetZ(point.Z()); }

			if (point.X() > urrPoint.X()) { urrPoint.SetX(point.X()); }
			if (point.Y() > urrPoint.Y()) { urrPoint.SetY(point.Y()); }
			if (point.Z() > urrPoint.Z()) { urrPoint.SetZ(point.Z()); }
		}
	}

	// compute diagonal distance
	double distance = lllPoint.Distance(urrPoint);

	return std::make_tuple( lllPoint, urrPoint, distance );
}

BoostPoint3D Point3DOTB(gp_Pnt oP){
	return BoostPoint3D(oP.X(), oP.Y(), oP.Z());
}

gp_Pnt Point3DBTO(BoostPoint3D oP) {
	return gp_Pnt(bg::get<0>(oP), bg::get<1>(oP), bg::get<2>(oP));
}

gp_Pnt getLowestPoint(TopoDS_Shape shape, bool areaFilter)
{
	double lowestZ = 9999;
	gp_Pnt lowestP(0, 0, 0);

	TopExp_Explorer expl;

	TopoDS_Shape largestFace = shape;

	if (areaFilter)
	{
		GProp_GProps gprop;
		largestFace;
		double area = 0;

		for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next())
		{
			BRepGProp::SurfaceProperties(TopoDS::Face(expl.Current()), gprop);
			if (area < gprop.Mass())
			{
				area = gprop.Mass();
				largestFace = TopoDS::Face(expl.Current());
			}
		}
	}

	for (expl.Init(largestFace, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() < lowestZ)
		{
			lowestZ = p.Z();
			lowestP = p;
		}
	}

	double sumX = 0;
	double sumY = 0;
	int aP = 0;

	for (expl.Init(largestFace, TopAbs_VERTEX); expl.More(); expl.Next())
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

gp_Pnt getHighestPoint(TopoDS_Shape shape)
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

	return gp_Pnt(sumX / aP, sumY / aP, highestZ);
}

std::vector<IfcSchema::IfcProduct*> getNestedProducts(IfcSchema::IfcProduct* product) {
	
	std::vector<IfcSchema::IfcProduct*> productList;

	if (!product->hasRepresentation())
	{
		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();

		if (decomposedProducts->size() == 0) { return {}; }

		for (auto et = decomposedProducts->begin(); et != decomposedProducts->end(); ++et) {
			IfcSchema::IfcRelAggregates* aggregates = *et;
			IfcSchema::IfcObjectDefinition::list::ptr aggDef = aggregates->RelatedObjects();

			for (auto rt = aggDef->begin(); rt != aggDef->end(); ++rt) {
				IfcSchema::IfcObjectDefinition* aggDef = *rt;
				productList.emplace_back(aggDef->as<IfcSchema::IfcProduct>());
			}
		}
	}
	else if (product->hasRepresentation())
	{
		productList.emplace_back(product);
	}

	return productList;
}

helper::helper(std::string path) {

	helper::findSchema(path);

	IfcParse::IfcFile*  sourceFile = new IfcParse::IfcFile(path);

	bool good = sourceFile->good();

	if (!good) 
	{ 
		std::cout << "Unable to parse .ifc file" << std::endl; 
	}
	else { 
		std::cout << "Valid IFC file found" << std::endl; 
		std::cout << std::endl;
		
		file_ = sourceFile;

		kernel_ = new IfcGeom::Kernel(file_);

		helper::setUnits(file_);
	}
}


void helper::findSchema(std::string path) {
	std::ifstream infile(path);
	std::string line;

	while (std::getline(infile, line))
	{
		if (line[0] == '#') {
			std::cout << "[Warning] No valid ifc scheme found" << std::endl;
			break; 
		}

		if (line.find("FILE_SCHEMA(('IFC4'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC4" << std::endl;
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC2X3'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC2X3" << std::endl;
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x1'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC4x1" << std::endl;
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x2'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC4x2" << std::endl;
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x3'))") != std::string::npos) {
			std::cout << "Valid scheme found: IFC4x3" << std::endl;
			break;
		}
	}
	infile.close();
}


std::vector<gp_Pnt> helper::getAllPoints(IfcSchema::IfcProduct::list::ptr products)
{
	std::vector<gp_Pnt> pointList;


	for (auto it = products->begin(); it != products->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;

		if (!product->hasRepresentation()) { continue; }
		if (product->data().type()->name() == "IfcAnnotation") { continue; } // find points another way
		if (product->data().type()->name() == "IfcSite") { continue; }
		if (product->data().type()->name() == "IfcBuildingElementProxy") { continue; }

		std::vector<gp_Pnt> temp = getObjectPoints(product);

		if (temp.size() > 1);
		{
			for (size_t i = 0; i < temp.size(); i++) { 
				pointList.emplace_back(temp[i]); 
			}
		}
		temp.clear();

	}

	return pointList;
}

bool helper::hasSetUnits() {
	if (!length_ || !area_ || !volume_){ return false; }
	else { return true;  }
}


void helper::setUnits(IfcParse::IfcFile *file) {
	double length = 0;
	double area = 0;
	double volume = 0;
	
	IfcSchema::IfcUnitAssignment::list::ptr presentUnits = file->instances_by_type<IfcSchema::IfcUnitAssignment>();
	if (presentUnits.get()->size() == 0) {
		std::cout << "[Error] No unit assignment has been found" << std::endl;
		return;
	}
	else if (presentUnits.get()->size() > 1)
	{
		std::cout << "[Error] Multiple unit assignments have been found" << std::endl;
		return;
	}


	for (IfcSchema::IfcUnitAssignment::list::it it = presentUnits->begin(); it != presentUnits->end(); ++it)
	{
		const IfcSchema::IfcUnitAssignment* itUnits = *it;
		auto units = itUnits->Units();

		for (auto et = units.get()->begin(); et != units.get()->end(); et++) {
			auto unit = *et;

			if (unit->declaration().type() == 902 || unit->declaration().type() == 765) // select the IfcSIUnit
			{
				std::string unitType = unit->data().getArgument(1)->toString();
				std::string SiUnitBase = unit->data().getArgument(3)->toString();
				std::string SiUnitAdd = unit->data().getArgument(2)->toString();

				if (unitType == ".LENGTHUNIT.")
				{
					if (SiUnitBase == ".METRE.") { length = 1; }
					if (SiUnitAdd == ".MILLI.") { length = length / 1000; }
				}
				else if (unitType == ".AREAUNIT.")
				{
					if (SiUnitBase == ".SQUARE_METRE.") { area = 1; }
					if (SiUnitAdd == ".MILLI.") { area = area / pow(1000, 2); }
				}
				if (unitType == ".VOLUMEUNIT.")
				{
					if (SiUnitBase == ".CUBIC_METRE.") { volume = 1; }
					if (SiUnitAdd == ".MILLI.") { volume = volume / pow(1000, 3); }
				}
			}
		}
	}

	// check if units have been found
	std::cout << "found units:" << std::endl;
	if (!length)
	{
		std::cout << "[Error] SI unit for length cannot be found!" << std::endl;
		return;
	}
	else if (length == 1) { std::cout << "- Lenght in metre" << std::endl; }
	else if (length == 0.001) { std::cout << "- Lenght in millimetre" << std::endl; }

	if (!area)
	{
		std::cout << "[Error] SI unit for area cannot be found!" << std::endl;
		return;
	}
	else if (area == 1) { std::cout << "- Area in square metre" << std::endl; }
	else if (area == 0.000001) { std::cout << "- Area in square millimetre" << std::endl; }

	if (!volume)
	{
		std::cout << "[Error] SI unit for volume cannot be found!" << std::endl;
		return;
	}
	else if (volume == 1) { std::cout << "- Volume in cubic metre" << std::endl; }
	else if (volume == 0.000000001) { std::cout << "- Volume in cubic millimetre" << std::endl; }

	std::cout << std::endl;

	//internalize the data
	length_ = length;
	area_ = area;
	volume_ = volume;
}

void helper::internalizeGeo()
{
	std::cout << "Internalizing Geometry of Construction Model\n" << std::endl;

	// get products
	IfcSchema::IfcProduct::list::ptr products = file_->instances_by_type<IfcSchema::IfcProduct>();

	std::vector<gp_Pnt> pointList = getAllPoints(products);

	// approximate smalles bbox
	double angle = 22.5 * (M_PI / 180);
	double rotation = 0;
	int maxIt = 15;

	// set data for a bbox
	auto base = rotatedBBoxDiagonal(pointList, rotation);
	gp_Pnt lllPoint = std::get<0>(base);
	gp_Pnt urrPoint = std::get<1>(base);
	double smallestDistance = std::get<2>(base);


	for (size_t i = 0; i < maxIt; i++)
	{
		std::tuple<gp_Pnt, gp_Pnt, double> left;
		std::tuple<gp_Pnt, gp_Pnt, double> right;

		left = rotatedBBoxDiagonal(pointList, rotation - angle);
		right = rotatedBBoxDiagonal(pointList, rotation + angle);
	
		if (std::get<2>(left) > std::get<2>(right) && smallestDistance > std::get<2>(right))
		{ 
			rotation = rotation + angle; 
			smallestDistance = std::get<2>(right);
			lllPoint = std::get<0>(right);
			urrPoint = std::get<1>(right);
		}
		else  if (smallestDistance > std::get<2>(left))
		{ 
			rotation = rotation - angle;
			smallestDistance = std::get<2>(left);
			lllPoint = std::get<0>(left);
			urrPoint = std::get<1>(left);
		}
		
		//make angle smaller
		angle = angle / 2;
	}

	lllPoint_ = lllPoint;
	urrPoint_ = urrPoint;
	originRot_ = rotation;

	hasGeo = true;
}

void helper::internalizeGeo(double angle) {
	std::cout << "Internalizing Geometry\n" << std::endl;

	IfcSchema::IfcProduct::list::ptr products = file_->instances_by_type<IfcSchema::IfcProduct>();
	std::vector<gp_Pnt> pointList = getAllPoints(products);

	auto bbox = rotatedBBoxDiagonal(pointList, angle);

	lllPoint_ = std::get<0>(bbox);
	urrPoint_ = std::get<1>(bbox);
	originRot_ = angle;

	hasGeo = true;
}

void helper::indexGeo()
{
	// this indexing is done based on the rotated bboxes of the objects
	// the bbox does thus comply with the model bbox but not with the actual objects original location

	// add the floorslabs to the rtree
	addObjectToIndex<IfcSchema::IfcSlab::list::ptr>(file_->instances_by_type<IfcSchema::IfcSlab>());
	addObjectToIndex<IfcSchema::IfcRoof::list::ptr>(file_->instances_by_type<IfcSchema::IfcRoof>());

	// add the walls to the rtree
	addObjectToIndex<IfcSchema::IfcWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcWall>());
	addObjectToIndex<IfcSchema::IfcCovering::list::ptr>(file_->instances_by_type<IfcSchema::IfcCovering>());

	// add the columns to the rtree TODO sweeps
	addObjectToIndex<IfcSchema::IfcColumn::list::ptr>(file_->instances_by_type<IfcSchema::IfcColumn>());

	// add the beams to the rtree
	addObjectToIndex<IfcSchema::IfcBeam::list::ptr>(file_->instances_by_type<IfcSchema::IfcBeam>());

	// add the curtain walls to the rtree
	addObjectToIndex<IfcSchema::IfcCurtainWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcCurtainWall>());
	addObjectToIndex<IfcSchema::IfcPlate::list::ptr>(file_->instances_by_type<IfcSchema::IfcPlate>());
	addObjectToIndex<IfcSchema::IfcMember::list::ptr>(file_->instances_by_type<IfcSchema::IfcMember>());

	// add doors to the rtree (for the appartment detection)
	addObjectToIndex<IfcSchema::IfcDoor::list::ptr>(file_->instances_by_type<IfcSchema::IfcDoor>());
	addObjectToIndex<IfcSchema::IfcWindow::list::ptr>(file_->instances_by_type<IfcSchema::IfcWindow>());

	// find valid voids
	applyVoids();

	// =================================================================================================
	// connectivity objects indexing
	addObjectToCIndex<IfcSchema::IfcDoor::list::ptr>(file_->instances_by_type<IfcSchema::IfcDoor>());
	addObjectToCIndex<IfcSchema::IfcStair::list::ptr>(file_->instances_by_type<IfcSchema::IfcStair>());

	// =================================================================================================
	// room objects indexing
	addObjectToRIndex<IfcSchema::IfcSpace::list::ptr>(file_->instances_by_type<IfcSchema::IfcSpace>());

	// =================================================================================================
	// opening indexing

}

bg::model::box < BoostPoint3D > helper::makeObjectBox(IfcSchema::IfcProduct* product)
{
	std::vector<gp_Pnt> productVert = getObjectPoints(product);
	if (!productVert.size() > 1) { return bg::model::box < BoostPoint3D >({0,0,0}, {0,0,0}); }

	// only outputs 2 corners of the three needed corners!
	auto box = rotatedBBoxDiagonal(productVert, originRot_);

	BoostPoint3D boostlllpoint = Point3DOTB(std::get<0>(box));
	BoostPoint3D boosturrpoint = Point3DOTB(std::get<1>(box));

	return bg::model::box < BoostPoint3D >(boostlllpoint, boosturrpoint);
}

bg::model::box < BoostPoint3D > helper::makeObjectBox(const std::vector<IfcSchema::IfcProduct*> products)
{
	std::vector<gp_Pnt> productVert;

	for (size_t i = 0; i < products.size(); i++)
	{
		std::vector<gp_Pnt> singleVerts = getObjectPoints(products[i]);

		for (size_t j = 0; j < singleVerts.size(); j++)
		{
			productVert.emplace_back(singleVerts[j]);
		}
	}

	if (!productVert.size() > 1) { return bg::model::box < BoostPoint3D >({ 0,0,0 }, { 0,0,0 }); }

	// only outputs 2 corners of the three needed corners!
	auto box = rotatedBBoxDiagonal(productVert, originRot_);

	BoostPoint3D boostlllpoint = Point3DOTB(std::get<0>(box));
	BoostPoint3D boosturrpoint = Point3DOTB(std::get<1>(box));

	return bg::model::box < BoostPoint3D >(boostlllpoint, boosturrpoint);
}

TopoDS_Solid helper::makeSolidBox(gp_Pnt lll, gp_Pnt urr, double angle)
{
	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;

	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid solidbox;
	brepBuilder.MakeSolid(solidbox);

	gp_Pnt p0(rotatePointWorld(lll, -angle));
	gp_Pnt p1 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()), -angle);
	gp_Pnt p2 = rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()), -angle);
	gp_Pnt p3 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()), -angle);

	gp_Pnt p4(rotatePointWorld(urr, -angle));
	gp_Pnt p5 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), urr.Z()), -angle);
	gp_Pnt p6 = rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), urr.Z()), -angle);
	gp_Pnt p7 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), urr.Z()), -angle);

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
	brepBuilder.Add(solidbox, brepSewer.SewedShape());

	return solidbox;
}

void helper::correctRooms() 
{
	// get all rooms in a cluser
	std::vector<roomLookupValue> roomValues = getFullRLookup();

	// compute volume of every room
	std::vector<double> roomVolume;
	GProp_GProps gprop;

	for (size_t i = 0; i < roomValues.size(); i++)
	{
		BRepGProp::VolumeProperties(std::get<1>(roomValues[i]), gprop);
		roomVolume.emplace_back(gprop.Mass());

		BRepClass3d_SolidClassifier insideChecker;
		insideChecker.Load(std::get<1>(roomValues[i]));
		gp_Pnt potentialCenter = gprop.CentreOfMass();

		insideChecker.Perform(potentialCenter, 0.001);

		//if (insideChecker.State())
		{
			roomCenterPoints_.emplace_back(gprop.CentreOfMass());
			continue;
		}



	}

	// check which room is complex
	for (size_t i = 0; i < roomValues.size(); i++)
	{
		// create bounding box around roomshape
		TopoDS_Shape roomShape = std::get<1>(roomValues[i]);
		std::vector<gp_Pnt> cornerPoints;

		gp_Pnt lll(9999, 9999, 9999);
		gp_Pnt urr(-9999, -9999, -9999);

		TopExp_Explorer expl;
		for (expl.Init(roomShape, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			cornerPoints.emplace_back(p);
		}

		for (size_t j = 0; j < cornerPoints.size(); j++)
		{
			auto currentCorner = cornerPoints[j];
			if (urr.X() < currentCorner.X()) { urr.SetX(currentCorner.X()); }
			if (urr.Y() < currentCorner.Y()) { urr.SetY(currentCorner.Y()); }
			if (urr.Z() < currentCorner.Z()) { urr.SetZ(currentCorner.Z()); }
			if (lll.X() > currentCorner.X()) { lll.SetX(currentCorner.X()); }
			if (lll.Y() > currentCorner.Y()) { lll.SetY(currentCorner.Y()); }
			if (lll.Z() > currentCorner.Z()) { lll.SetZ(currentCorner.Z()); }
		}
		// check if other rooms fall inside evaluating room
		boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(lll), Point3DOTB(urr));

		std::vector<Value> qResult;
		qResult.clear();
		getRoomIndexPointer()->query(bgi::intersects(qBox), std::back_inserter(qResult));

		if (qResult.size() == 0) { continue; }

		bool complex = false;

		BRepClass3d_SolidClassifier insideChecker;
		insideChecker.Load(roomShape);

		for (size_t j = 0; j < qResult.size(); j++)
		{
			if (i == qResult[j].second) { continue; } // ignore self

			TopoDS_Shape matchingRoom = std::get<1>(roomValues[qResult[j].second]);
			for (expl.Init(matchingRoom, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);
				insideChecker.Perform(p, 0.01);
				if (!insideChecker.State() || insideChecker.IsOnAFace())
				{
					if (roomVolume[i] > roomVolume[qResult[j].second])
					{
						complex = true;
						continue;
					}
				}
				if (complex)
				{
					complex = false;
					break;
				}
			}
			if (complex) {
				std::get<0>(roomValues[i])->setCompositionType(IfcSchema::IfcElementCompositionEnum::IfcElementComposition_COMPLEX);
				break;
			}
		}
		if (!complex)
		{
			std::get<0>(roomValues[i])->setCompositionType(IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT);
			gp_Pnt centerOfMass = gprop.CentreOfMass();
		}
	}
}

std::vector<std::vector<gp_Pnt>> helper::triangulateProduct(IfcSchema::IfcProduct* product)
{
	std::vector<TopoDS_Face> faceList = getObjectFaces(product, true);

	std::vector<std::vector<gp_Pnt>> triangleMeshList;

	for (size_t i = 0; i < faceList.size(); i++)
	{
		BRepMesh_IncrementalMesh faceMesh = BRepMesh_IncrementalMesh(faceList[i], 0.004);

		TopLoc_Location loc;
		auto mesh = BRep_Tool::Triangulation(faceList[i], loc);
		const gp_Trsf& trsf = loc.Transformation();

		if (mesh.IsNull()) //TODO warning
		{
			continue;
		}

		for (size_t j = 1; j <= mesh.get()->NbTriangles(); j++)
		{
			const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

			std::vector<gp_Pnt> trianglePoints;
			for (size_t k = 1; k <= 3; k++)
			{
				gp_Pnt p = mesh->Nodes().Value(theTriangle(k)).Transformed(trsf);
				trianglePoints.emplace_back(p);
			}

			triangleMeshList.emplace_back(trianglePoints);
		}

	}

	return triangleMeshList;
}

template <typename T>
void helper::addObjectToIndex(T object) {
	// add doors to the rtree (for the appartment detection)
	for (auto it = object->begin(); it != object->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;		

		bg::model::box <BoostPoint3D> box = makeObjectBox(product);
		if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
			bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
			std::cout << "Failed: " + product->data().toString() << std::endl;
			continue;
		}

		TopoDS_Shape shape = getObjectShape(product);
		TopoDS_Shape cbbox;
		bool hasCBBox = false;

		//TODO get shape

		if (product->data().type()->name() == "IfcDoor" || product->data().type()->name() == "IfcWindow")
		{
			// get potential nesting objects
			std::vector<Value> qResult;
			qResult.clear();
			index_.query(bgi::intersects(box), std::back_inserter(qResult));

			//printPoint(box.max_corner());
			//printPoint(box.min_corner());

			BRepExtrema_DistShapeShape distanceMeasurer;
			distanceMeasurer.LoadS1(shape);

			bool matchFound = false;
			IfcSchema::IfcProduct* matchingUntrimmedProduct;
			TopoDS_Shape matchingUntrimmedShape;

			for (size_t i = 0; i < qResult.size(); i++)
			{
				LookupValue lookup = productLookup_[qResult[i].second];
				IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);

				if (qProduct->data().type()->name() != "IfcWall" && 
					qProduct->data().type()->name() != "IfcWallStandardCase" &&
					qProduct->data().type()->name() != "IfcRoof" && 
					qProduct->data().type()->name() != "IfcSlab")
				{
					continue;
				}

				auto search = adjustedshapeLookup_.find(qProduct->data().id());
				if (search == adjustedshapeLookup_.end()) { continue; }

				TopoDS_Shape qUntrimmedShape = search->second;

				TopExp_Explorer expl;
				for (expl.Init(qUntrimmedShape, TopAbs_SOLID); expl.More(); expl.Next()) {

					// get distance to isolated actual object
					distanceMeasurer.LoadS2(expl.Current());
					distanceMeasurer.Perform();
					double distance = distanceMeasurer.Value();

					if (!distanceMeasurer.InnerSolution()) { continue; }
					if (distance > 0.2) { continue; }

					matchFound = true;
					matchingUntrimmedProduct = qProduct;
					matchingUntrimmedShape = qUntrimmedShape;

					break;
				}
				if (matchFound) { break; }
			}


			if (!matchFound)
			{
				// if no void was found
				// find longest horizontal and vertical edge
				std::vector<gp_Pnt> pointList;
				std::vector<gp_Pnt> horizontalMaxEdge;
				std::vector<gp_Pnt> verticalMaxEdge;

				double maxHDistance = 0;
				double maxVDistance = 0;

				TopExp_Explorer expl;
				for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
				{
					TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
					gp_Pnt p = BRep_Tool::Pnt(vertex);
					pointList.emplace_back(p);
				}

				for (size_t i = 0; i < pointList.size(); i+=2)
				{
					gp_Pnt p1 = pointList[i];
					gp_Pnt p2 = pointList[i + 1];

					double vDistance = abs(p1.Z() - p2.Z());

					if (maxVDistance < vDistance)
					{
						maxVDistance = vDistance;
						verticalMaxEdge = { p1, p2 };
					}

					p1.SetZ(0);
					p2.SetZ(0);

					double hDistance = p1.Distance(p2);

					if (hDistance > maxHDistance)
					{
						maxHDistance = hDistance;
						horizontalMaxEdge = { p1, p2 };
					}
				}

				// get rotation in xy plane
				gp_Pnt bp(0, 0, 0);

				double bpDistance0 = bp.Distance(horizontalMaxEdge[0]);
				double bpDistance1 = bp.Distance(horizontalMaxEdge[1]);

				if (bpDistance0 > bpDistance1)
				{
					std::reverse(horizontalMaxEdge.begin(), horizontalMaxEdge.end());
				}

				gp_Pnt p1 = horizontalMaxEdge[0];
				gp_Pnt p2 = horizontalMaxEdge[1];

				double angle = 0;

				if (abs(p1.Y() - p2.Y()) > 0.00001)
				{
					double os = (p1.Y() - p2.Y()) / p1.Distance(p2);
					angle = asin(os);
				}

				auto base = rotatedBBoxDiagonal(pointList, angle);
				auto base2 = rotatedBBoxDiagonal(pointList, -angle);

				gp_Pnt lllPoint = std::get<0>(base);
				gp_Pnt urrPoint = std::get<1>(base);
				double rot = angle;

				if (lllPoint.Distance(urrPoint) > std::get<0>(base2).Distance(std::get<1>(base2)))
				{
					lllPoint = std::get<0>(base2);
					urrPoint = std::get<1>(base2);

					rot = -angle;
				}				

				hasCBBox = true;
				cbbox = makeSolidBox(lllPoint, urrPoint, rot);

				// TODO get rotation in z plane

			}

		}

		index_.insert(std::make_pair(box, (int)index_.size()));
		std::vector<std::vector<gp_Pnt>> triangleMeshList = triangulateProduct(product);
		auto lookup = std::make_tuple(*it, triangleMeshList, shape, hasCBBox, cbbox);
		productLookup_.emplace_back(lookup);
	}
}

template <typename T>
void helper::addObjectToCIndex(T object) {
	// add doors to the rtree (for the appartment detection)
	for (auto it = object->begin(); it != object->end(); ++it) {

		IfcSchema::IfcProduct* product = *it;
		std::vector<IfcSchema::IfcProduct*> productList = getNestedProducts(product);

		bg::model::box <BoostPoint3D> box = makeObjectBox(productList);
		if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
			bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
			continue;
		}

		gp_Pnt groundObjectPoint = { 0, 0,9999 };
		gp_Pnt topObjectPoint = {0, 0, -9999};
		bool isDoor = false;

		if (product->data().type()->name() == "IfcDoor") { isDoor = true; }

		for (size_t i = 0; i < productList.size(); i++)
		{
			TopoDS_Shape productShape = getObjectShape(productList[i]);
			gp_Pnt potGround = getLowestPoint(productShape, isDoor);
			gp_Pnt potTop = getHighestPoint(productShape);

			if (!isDoor)
			{
				if (potGround.Z() < groundObjectPoint.Z()) { groundObjectPoint = potGround; }
				if (potTop.Z() > topObjectPoint.Z()) { topObjectPoint = potTop; }
			}
			if (isDoor)
			{
				if (potGround.Z() < groundObjectPoint.Z()) { groundObjectPoint = potGround; }
				if (potTop.Z() > topObjectPoint.Z()) { topObjectPoint = potTop; }
			}
		}

		std::vector<gp_Pnt> connectingPoints = { groundObjectPoint, topObjectPoint };

		cIndex_.insert(std::make_pair(box, (int)cIndex_.size()));
		std::vector<roomObject*>* space = new std::vector<roomObject*>;
		auto lookup = std::make_tuple(*it, connectingPoints, space);
		connectivityLookup_.emplace_back(lookup);
	}
}

template<typename T>
void helper::addObjectToRIndex(T object){
	// add doors to the rtree (for the appartment detection)
	for (auto it = object->begin(); it != object->end(); ++it) {
		bg::model::box <BoostPoint3D> box = makeObjectBox(*it);
		if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
			bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
			continue;
		}
		rIndex_.insert(std::make_pair(box, (int)rIndex_.size()));
		TopoDS_Shape spaceShape = getObjectShape(*it);
		auto lookup = std::make_tuple(*it, spaceShape);
		roomLookup_.emplace_back(lookup);
	}
}



IfcSchema::IfcOwnerHistory* helper::getHistory()
{
	IfcSchema::IfcOwnerHistory* ownerHistory;
	IfcSchema::IfcOwnerHistory::list::ptr ownerHistories = file_->instances_by_type<IfcSchema::IfcOwnerHistory>();

	if (ownerHistories.get()->size() != 1) { std::cout << "[Warning] multiple owners objects found!" << std::endl; }

	return *ownerHistories.get()->begin();
}

std::vector<gp_Pnt> helper::getObjectPoints(IfcSchema::IfcProduct* product, bool sortEdges, bool simple)
{
	std::vector<gp_Pnt> pointList;

	//std::cout << product->data().toString() << std::endl;
	if (!product->hasRepresentation()) { 

		std::vector<IfcSchema::IfcProduct*> productList;

		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();

		if (decomposedProducts->size() == 0) { return { gp_Pnt(0.0, 0.0, 0.0) }; }

		for (auto et = decomposedProducts->begin(); et != decomposedProducts->end(); ++et) {
			IfcSchema::IfcRelAggregates* aggregates = *et;
			IfcSchema::IfcObjectDefinition::list::ptr aggDef = aggregates->RelatedObjects();

			for (auto rt = aggDef->begin(); rt != aggDef->end(); ++rt) {
				IfcSchema::IfcObjectDefinition* aggDef = *rt;
				productList.emplace_back(aggDef->as<IfcSchema::IfcProduct>());
			}
		}

		for (size_t i = 0; i < productList.size(); i++)
		{
			TopoDS_Shape rShape = getObjectShape(productList[i]);
			TopExp_Explorer expl;
			for (expl.Init(rShape, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);
				//pointList.emplace_back(p);
			}
		}
	}
	else {

		//std::cout << product->data().toString() << std::endl;

		TopoDS_Shape rShape = getObjectShape(product, simple);

		TopExp_Explorer expl;
		for (expl.Init(rShape, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			pointList.emplace_back(p);
		}
	}


	if (!sortEdges) { return pointList; }

	std::vector<gp_Pnt> pointListSmall;
	for (size_t i = 0; i < pointList.size(); i++)
	{
		if (i%2 == 0) { pointListSmall.emplace_back(pointList[i]); }
	}

	return pointListSmall;

}

std::vector<TopoDS_Face> helper::getObjectFaces(IfcSchema::IfcProduct* product, bool simple)
{
	std::vector<TopoDS_Face> faceList;

	if (!product->hasRepresentation()) { return {}; }

	TopoDS_Shape rShape = getObjectShape(product, simple);

	TopExp_Explorer expl;
	for (expl.Init(rShape, TopAbs_FACE); expl.More(); expl.Next())
	{
		TopoDS_Face face = TopoDS::Face(expl.Current());
		faceList.emplace_back(face);
	}

	return faceList;
}

TopoDS_Shape helper::getObjectShape(IfcSchema::IfcProduct* product, bool adjusted)
{
	// filter with lookup
	if (product->data().type()->name() != "IfcWall" &&
		product->data().type()->name() != "IfcWallStandardCase" && 
		product->data().type()->name() != "IfcRoof")
	{
		adjusted = false;
	}

	if (!adjusted)
	{
		auto search = shapeLookup_.find(product->data().id());
		if (search != shapeLookup_.end())
		{
			return search->second;
		}
	}

	if (adjusted)
	{
		auto search = adjustedshapeLookup_.find(product->data().id());
		if (search != adjustedshapeLookup_.end())
		{
			return search->second;
		}
	}

	// find representations
	std::vector<IfcSchema::IfcProduct*> productList;

	if (!product->hasRepresentation()) { 

		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();

		if (decomposedProducts->size() == 0) { return { }; }

		for (auto et = decomposedProducts->begin(); et != decomposedProducts->end(); ++et) {
			IfcSchema::IfcRelAggregates* aggregates = *et;
			IfcSchema::IfcObjectDefinition::list::ptr aggDef = aggregates->RelatedObjects();

			for (auto rt = aggDef->begin(); rt != aggDef->end(); ++rt) {
				IfcSchema::IfcObjectDefinition* aggDef = *rt;

				IfcSchema::IfcProduct* addprod = aggDef->as<IfcSchema::IfcProduct>();

				if (addprod->hasRepresentation())
				{
					productList.emplace_back(addprod);
				}

				
			}
		}
		return {}; 
	}
	else {
		productList.emplace_back(product);
	}

	int id = product->data().id();
	
	BRep_Builder builder;
	TopoDS_Compound collection;
	TopoDS_Compound simpleCollection;
	builder.MakeCompound(collection);
	builder.MakeCompound(simpleCollection);

	bool hasHoles = false;
	bool isFloor = false;

	if (product->data().type()->name() == "IfcWall" ||
		product->data().type()->name() == "IfcWallStandardCase" ||
		product->data().type()->name() == "IfcRoof" ) { hasHoles = true; }
	if (product->data().type()->name() == "IfcSlab") { isFloor = true; }

	for (size_t i = 0; i < productList.size(); i++)
	{
		TopoDS_Compound comp;

		IfcSchema::IfcRepresentation* ifc_representation = 0;

		IfcSchema::IfcProductRepresentation* prodrep = product->Representation();
		IfcSchema::IfcRepresentation::list::ptr reps = prodrep->Representations();

		for (IfcSchema::IfcRepresentation::list::it it = reps->begin(); it != reps->end(); ++it) {
			IfcSchema::IfcRepresentation* rep = *it;
			if (rep->RepresentationIdentifier() == "Body") {
				ifc_representation = rep;
				break;
			}
		}

		if (ifc_representation == 0)
		{
			for (IfcSchema::IfcRepresentation::list::it it = reps->begin(); it != reps->end(); ++it) {
				IfcSchema::IfcRepresentation* rep = *it;

				if (rep->RepresentationIdentifier() == "Annotation") {
					ifc_representation = rep;
					break;
				}
			}
		}


		IfcGeom::IteratorSettings settings;
		if (isFloor) { settings.set(settings.DISABLE_OPENING_SUBTRACTIONS, true); }

		if (!ifc_representation)
		{
			return {};
		}

		if (ifc_representation->RepresentationIdentifier() == "Annotation")
		{
			gp_Trsf trsf;
			kernel_->convert_placement(product->ObjectPlacement(), trsf);

			IfcSchema::IfcRepresentationItem::list::ptr representationItems = ifc_representation->Items();

			for (auto it = representationItems->begin(); it != representationItems->end(); ++it)
			{
				IfcSchema::IfcRepresentationItem* representationItem = *it;

				if (representationItem->data().type()->name() == "IfcTextLiteralWithExtent") { continue; }				
				
			}


			//std::cout << representationItems->data().toString() << std::endl;

			// data is never deleted, can be used later as internalized data
			//IfcGeom::IfcRepresentationShapeItems ob(kernel_->convert(representationItems));



		}
		else if (ifc_representation->RepresentationIdentifier() == "Body")
		{
			gp_Trsf placement;
			gp_Trsf trsf;

			kernel_->convert_placement(product->ObjectPlacement(), trsf);
			IfcGeom::BRepElement<double, double>* brep = kernel_->convert(settings, ifc_representation, product);
			kernel_->convert_placement(ifc_representation, placement);

			comp = brep->geometry().as_compound();
			comp.Move(trsf* placement); // location in global space

			builder.Add(collection, comp);

			if (hasHoles)
			{
				settings.set(settings.DISABLE_OPENING_SUBTRACTIONS, true);
				brep = kernel_->convert(settings, ifc_representation, product);
				kernel_->convert_placement(ifc_representation, placement);

				comp = brep->geometry().as_compound();
				comp.Move(trsf * placement); // location in global space

				builder.Add(simpleCollection, comp);
			}
		}
	}

	shapeLookup_[product->data().id()] = collection;

	if (hasHoles)
	{
		adjustedshapeLookup_[product->data().id()] = simpleCollection;
		
	}

	if (adjusted)
	{
		return simpleCollection;
	}
	
	return collection;
}

void helper::updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape, bool adjusted)
{
	if (!product->hasRepresentation()) { return; }

	// filter with lookup
	if (!adjusted)
	{
		auto search = shapeLookup_.find(product->data().id());
		if (search == shapeLookup_.end())
		{
			return;
		}

		shapeLookup_[product->data().id()] = shape;
	}

	if (adjusted)
	{
		auto search = adjustedshapeLookup_.find(product->data().id());
		if (search == adjustedshapeLookup_.end())
		{
			return;
		}

		adjustedshapeLookup_[product->data().id()] = shape;
	}
}

void helper::updateIndex(IfcSchema::IfcProduct* product, TopoDS_Shape shape) {
	
	auto base = rotatedBBoxDiagonal(getObjectPoints(product), 0);
	gp_Pnt lllPoint = std::get<0>(base);
	gp_Pnt urrPoint = std::get<1>(base);

	std::vector<Value> qResult;
	boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(lllPoint), Point3DOTB(urrPoint));

	index_.query(bgi::intersects(qBox), std::back_inserter(qResult));

	for (size_t i = 0; i < qResult.size(); i++)
	{
		LookupValue lookup = getLookup(qResult[i].second);
		IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);

		if (qProduct->data().id() != product->data().id())
		{
			continue;
		}

		std::vector<std::vector<gp_Pnt>> triangleMeshList = triangulateProduct(product);

		updateLookupTriangle(triangleMeshList, qResult[i].second);
	}
}

void helper::applyVoids()
{
	voidShapeAdjust<IfcSchema::IfcWallStandardCase::list::ptr>(file_->instances_by_type<IfcSchema::IfcWallStandardCase>());
	voidShapeAdjust<IfcSchema::IfcWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcWall>());
}

template <typename T>
void helper::voidShapeAdjust(T products) 
{
	for (auto it = products->begin(); it != products->end(); ++it) {
		IfcSchema::IfcProduct* wallProduct = *it;
		TopoDS_Shape untrimmedWallShape = getObjectShape(wallProduct, true);

		TopExp_Explorer expl;

		// get the voids
		IfcSchema::IfcElement* wallElement = wallProduct->as<IfcSchema::IfcElement>();
		IfcSchema::IfcRelVoidsElement::list::ptr voidElementList = wallElement->HasOpenings();
		std::vector<TopoDS_Shape> validVoidShapes;

		// find if the voids are filled or not
		for (auto et = voidElementList->begin(); et != voidElementList->end(); ++et) {
			IfcSchema::IfcRelVoidsElement* voidElement = *et;
			IfcSchema::IfcFeatureElementSubtraction* openingElement = voidElement->RelatedOpeningElement();
			TopoDS_Shape substractionShape = getObjectShape(openingElement);

			//printFaces(substractionShape);
			
			auto base = rotatedBBoxDiagonal(getObjectPoints(openingElement), 0);
			gp_Pnt lllPoint = std::get<0>(base);
			gp_Pnt urrPoint = std::get<1>(base);

			//printPoint(lllPoint);
			//printPoint(urrPoint);

			std::vector<Value> qResult;
			boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(lllPoint), Point3DOTB(urrPoint));

			//printPoint(lllPoint);
			//printPoint(urrPoint);

			index_.query(bgi::intersects(qBox), std::back_inserter(qResult));

			if (qResult.size() == 0)
			{
				validVoidShapes.emplace_back(substractionShape);
				continue;
			}

			BRepClass3d_SolidClassifier insideChecker;
			insideChecker.Load(substractionShape);

			bool inter = false;

			for (size_t i = 0; i < qResult.size(); i++)
			{
				LookupValue lookup = getLookup(qResult[i].second);
				IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);

				if (qProduct->data().type()->name() != "IfcWindow" &&
					qProduct->data().type()->name() != "IfcDoor" &&
					qProduct->data().type()->name() != "IfcColumn")
				{
					continue;
				}

				TopoDS_Shape qShape = getObjectShape(qProduct);

				TopExp_Explorer expl2;
				for (expl2.Init(qShape, TopAbs_VERTEX); expl2.More(); expl2.Next()) {

					insideChecker.Perform(BRep_Tool::Pnt(TopoDS::Vertex(expl2.Current())), 0.01);

					if (insideChecker.State() || insideChecker.IsOnAFace()) {
						inter = true;
						break;
					}
				}


				if (inter)
				{
					break;
				}
			}

			if (inter == false)
			{
				validVoidShapes.emplace_back(substractionShape);
				continue;
			}		
		}

		if (validVoidShapes.size() == 0 )
		{
			continue;
		}
		else if (validVoidShapes.size() > 0 && voidElementList->size() > 0) {
			// get a basepoint of the wall
			std::vector<gp_Pnt> pList;;
			for (expl.Init(untrimmedWallShape, TopAbs_VERTEX); expl.More(); expl.Next()) {
				pList.emplace_back(BRep_Tool::Pnt(TopoDS::Vertex(expl.Current())));
			}

			// bool out voidShape
			BOPAlgo_Splitter aSplitter;
			TopTools_ListOfShape aLSObjects;
			aLSObjects.Append(untrimmedWallShape);
			TopTools_ListOfShape aLSTools;

			TopExp_Explorer expl;
			for (size_t i = 0; i < validVoidShapes.size(); i++)
			{
				aLSTools.Append(validVoidShapes[i]);
			}

			aLSTools.Reverse();
			aSplitter.SetArguments(aLSObjects);
			aSplitter.SetTools(aLSTools);
			aSplitter.SetRunParallel(Standard_True);
			aSplitter.SetNonDestructive(Standard_True);

			aSplitter.Perform();

			const TopoDS_Shape& aResult = aSplitter.Shape(); // result of the operation

			TopoDS_Shape finalShape;
			std::vector<TopoDS_Solid> solids;
			for (expl.Init(aResult, TopAbs_SOLID); expl.More(); expl.Next()) {
				solids.emplace_back(TopoDS::Solid(expl.Current()));
			}

			for (size_t i = 0; i < pList.size(); i++)
			{
				int count = 0;

				for (size_t j = 0; j < solids.size(); j++)
				{
					for (expl.Init(solids[j], TopAbs_VERTEX); expl.More(); expl.Next()) {
						gp_Pnt evalPoint = BRep_Tool::Pnt(TopoDS::Vertex(expl.Current()));
						if (pList[i].X() == evalPoint.X() &&
							pList[i].Y() == evalPoint.Y() &&
							pList[i].Z() == evalPoint.Z() )
						{
							finalShape = solids[j];
							count ++;
							break;
						}
					}

				}

				if (count == 1)
				{
					break;
				}
			}
			updateShapeLookup(wallProduct, finalShape, true);
			updateIndex(wallProduct, finalShape);
		}
	}
}

void helper::whipeObject(IfcSchema::IfcProduct* product)
{
	file_->removeEntity(product);

	// TODO check dependencies!
}

void helper::wipeObject(helper* data, int id)
{
	//TODO find the nested indx located in the file
	//TODO find if the nested idnx have other dependencies 
	//TODO remove objects at indx that do not have dependencies
}

void helper::writeToFile(std::string path)
{
	std::ofstream storageFile;
	storageFile.open(path);
	std::cout << "[INFO] Exporting file " << fileName_ << std::endl;
	storageFile << *file_;
	std::cout << "[INFO] Exported succesfully" << std::endl;
	storageFile.close();
}

void helperCluster::internaliseData()
{

	bool debug = false;

	if (!helperList[0]->getDepending())
	{
		helperList[0]->internalizeGeo();

		lllPoint_ = helperList[0]->getLllPoint();
		urrPoint_ = helperList[0]->getUrrPoint();
		originRot_ = helperList[0]->getRotation();


		if (debug)
		{
			std::cout << "cluster:" << std::endl;
			printPoint(lllPoint_);
			printPoint(urrPoint_);
			std::cout << originRot_ << std::endl;
		}

		return;
	}

	for (size_t i = 0; i < size_; i++)
	{
		auto helper = helperList[i];

		if (helper->getIsConstruct())
		{
			helper->internalizeGeo();
			lllPoint_ = helper->getLllPoint();
			urrPoint_ = helper->getUrrPoint();
			originRot_ = helper->getRotation();
		}
	}

	for (size_t i = 0; i < size_; i++)
	{
		auto helper = helperList[i];

		if (!helper->getHasGeo())
		{
			helper->internalizeGeo(originRot_);

			// update bbox if needed
			gp_Pnt addLllPoint = helper->getLllPoint();
			gp_Pnt addUrrPoint = helper->getUrrPoint();

			if (addLllPoint.X() < lllPoint_.X()) { lllPoint_.SetX(addLllPoint.X()); }
			if (addLllPoint.Y() < lllPoint_.Y()) { lllPoint_.SetY(addLllPoint.Y()); }
			if (addLllPoint.Z() < lllPoint_.Z()) { lllPoint_.SetZ(addLllPoint.Z()); }

			if (addUrrPoint.X() > urrPoint_.X()) { urrPoint_.SetX(addUrrPoint.X()); }
			if (addUrrPoint.Y() > urrPoint_.Y()) { urrPoint_.SetY(addUrrPoint.Y()); }
			if (addUrrPoint.Z() > urrPoint_.Z()) { urrPoint_.SetZ(addUrrPoint.Z()); }
		}
	}

	hasBbox_ = true;

	if (debug)
	{
		std::cout << "cluster:" << std::endl;
		printPoint(lllPoint_);
		printPoint(urrPoint_);
		std::cout << originRot_ << std::endl;
	}
}

void helperCluster::appendHelper(helper* data)
{
	helperList.emplace_back(data);
	size_++;
}