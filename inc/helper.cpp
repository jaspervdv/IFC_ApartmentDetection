#include "helper.h"

std::tuple<gp_Pnt, gp_Pnt, double> rotatedBBoxDiagonal(std::vector<gp_Pnt> pointList, double angle) {
	
	bool isPSet = false;
	gp_Pnt lllPoint;
	gp_Pnt urrPoint;

	for (size_t i = 0; i < pointList.size(); i++)
	{
		gp_Pnt basePoint = pointList[i];
		double pX = basePoint.X();
		double pY = basePoint.Y();
		double pZ = basePoint.Z();

		gp_Pnt point = gp_Pnt(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);

		//std::cout << basePoint.X() << ", " << basePoint.Y() << ", " << basePoint.Z() << std::endl;
		//std::cout << point.X() << ", " << point.Y() << ", " << point.Z() << std::endl;

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

		internalizeGeo();
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
	std::cout << "Internalizing Geometry\n" << std::endl;

	std::vector<gp_Pnt> pointList;



	// get products
	IfcSchema::IfcProduct::list::ptr products = file_->instances_by_type<IfcSchema::IfcProduct>();

	for (auto it = products->begin(); it != products->end(); ++it) {

		IfcSchema::IfcProduct* product = *it;

		if (!product->hasRepresentation()) { continue; }
		if (product->data().type()->name() == "IfcSite") { continue; }

		auto representations = product->Representation()->Representations();

		gp_Trsf trsf;
		kernel_->convert_placement(product->ObjectPlacement(), trsf);

		for (auto et = representations.get()->begin(); et != representations.get()->end(); et++) {
			const IfcSchema::IfcRepresentation* representation = *et;

			std::string geotype = representation->data().getArgument(2)->toString();
			if (representation->data().getArgument(1)->toString() != "'Body'") { continue; }

			IfcSchema::IfcRepresentationItem* representationItems = *representation->Items().get()->begin();

			// data is never deleted, can be used later as internalized data
			std::unique_ptr<IfcGeom::IfcRepresentationShapeItems> ob = std::make_unique<IfcGeom::IfcRepresentationShapeItems>(kernel_->convert(representationItems));

			// move to OpenCASCADE
			const TopoDS_Shape rShape = ob.get()->at(0).Shape();
			const TopoDS_Shape aShape = rShape.Moved(trsf); // location in global space


			TopExp_Explorer expl;
			for (expl.Init(aShape, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				pointList.emplace_back(BRep_Tool::Pnt(vertex));
			}
		}
	}
	
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
}

IfcSchema::IfcOwnerHistory* helper::getHistory()
{
	IfcSchema::IfcOwnerHistory* ownerHistory;
	IfcSchema::IfcOwnerHistory::list::ptr ownerHistories = file_->instances_by_type<IfcSchema::IfcOwnerHistory>();

	if (ownerHistories.get()->size() != 1) { std::cout << "[Warning] multiple owners objects found!" << std::endl; }

	return *ownerHistories.get()->begin();
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