// TODO: Multiple schemas
#define IfcSchema Ifc4


// basic includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

// boost includes
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

// openCASCADE includes
#include <TopoDS.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

// IfcOpenShell includes
#include <ifcparse/IfcFile.h>
#include <ifcparse/IfcHierarchyHelper.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>

#if USE_VLD
#include <vld.h>
#endif

int main(int argc, char** argv) {
	Logger::SetOutput(&std::cout, &std::cout);

	std::string sourcePath = "D:/Documents/Uni/Thesis/sources/Models/AC20-FZK-Haus.ifc";
	std::string exportPath = "D:/Documents/Uni/Thesis/sources/Models/exports/AC20-FZK-Haus.ifc";

	//find schema of file
	std::ifstream infile(sourcePath);
	std::string line;
	while (std::getline(infile, line)) 
	{
		if (line[0] == '#') {
			break;
		}

		if (line.find("FILE_SCHEMA(('IFC4'))") != std::string::npos) {
			std::cout << "Valid scheme" << std::endl;
			break;
		} else if (line.find("FILE_SCHEMA(('IFC2X3'))") != std::string::npos) {
			//TODO translate IFC2x3 to IFC4
			break;
		}
	
	}
	infile.close();

	// get source file
	IfcParse::IfcFile* SourceFile = new IfcParse::IfcFile(sourcePath);

	if (!SourceFile->good()) {
		std::cout << "Unable to parse .ifc file" << std::endl;
		return 1;
	}
	else {
		std::cout << "Valid IFC file found" << std::endl;
	}

	std::cout << std::endl;
	IfcGeom::Kernel my_kernel(SourceFile);

	IfcSchema::IfcProduct::list::ptr prods = SourceFile->instances_by_type<IfcSchema::IfcProduct>();
	for (IfcSchema::IfcProduct::list::it it = prods->begin(); it != prods->end(); ++it) {
		IfcSchema::IfcProduct* prod = *it;

		//gp_Trsf trsf = my_kernel.convert(prod->ObjectPlacement());
		//const gp_XYZ& xyz = trsf.TranslationPart();
		//std::cout << prod->GlobalId() << ": " <<	xyz.X() << " " << xyz.Y() << " " << xyz.Z() << std::endl;
	}



	// create working basefile
	IfcHierarchyHelper<IfcSchema> workingFile;
	workingFile.header().file_name().name("test.ifc");

	std::vector<IfcUtil::IfcBaseClass*> area_ref;

	IfcSchema::IfcBuildingElement::list::ptr elements = SourceFile->instances_by_type<IfcSchema::IfcBuildingElement>();
	std::cout << "Found " << elements->size() << " elements " << std::endl;

	for (IfcSchema::IfcBuildingElement::list::it it = elements->begin(); it != elements->end(); ++it) {
		const IfcSchema::IfcBuildingElement* element = *it;

		const IfcSchema::IfcSlab* slab;

		// select all the slab objects
		if ((slab = element->as<IfcSchema::IfcSlab>()) != 0) {
			// get the IfcShapeRepresentation
			auto slabProduct = slab->Representation()->Representations();

			std::cout << slab->data().toString() << std::endl;

			for (auto et = slabProduct.get()->begin(); et != slabProduct.get()->end(); et++) {
				const IfcSchema::IfcRepresentation* slabRepresentation = *et;

				std::cout << slabRepresentation->data().toString() << std::endl;

				// select the body of the slabs (ignore the bounding boxes)
				if (slabRepresentation->data().getArgument(1)->toString() == "'Body'")
				{
					// select the geometry format
					auto slabItems = slabRepresentation->Items();

					for (auto st = slabItems.get()->begin(); st != slabItems.get()->end(); st++)
					{
						IfcSchema::IfcRepresentationItem* slabItem = *st;

						auto ob = my_kernel.convert(slabItem);

						// move to OpenCASCADE
						const TopoDS_Shape shape = ob[0].Shape();

						TopLoc_Location origin;


						//shape.Located();

						// set variables for top face selection
						TopoDS_Face topFace;
						double topHeight = -9999;
						int faceCount = 0;

						// loop through all faces of slab
						TopExp_Explorer expl;
						for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next())
						{
							faceCount++;
							TopoDS_Face face = TopoDS::Face(expl.Current());
							BRepAdaptor_Surface brepAdaptorSurface(face, Standard_True);
							// TODO determine horizontal face
							// TODO how to deal with not flat surfaces

							// select floor top face
							double faceHeight = face.Location().Transformation().TranslationPart().Z();

							std::cout << faceHeight << std::endl;
							if (faceHeight > topHeight)
							{
								topHeight = faceHeight;
								topFace = face;
							}
						}

						std::cout << topHeight << std::endl;

						// help for flatness determening
						int tcount = 0;
						int hcount = 0;
						std::vector<double> detectedHeight = std::vector<double>();

						// select points of the top face
						for (expl.Init(topFace, TopAbs_VERTEX); expl.More(); expl.Next()) 
						{
							TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());

							// untranslated point!
							gp_Pnt pt = BRep_Tool::Pnt(vertex);
							double xyz_z = pt.Z();

							if (!std::count(detectedHeight.begin(), detectedHeight.end(), xyz_z)) {detectedHeight.emplace_back(xyz_z);}

							//TODO translate
							std::cout << "(" << pt.X() << ", " << pt.Y() << ", " << pt.Z() << ")" << std::endl;

							tcount++;
						}

						if (detectedHeight.size() > 1){std::cout << "not flat" << std::endl;}

						//TODO select the floor level
						//TODO pair floors on the same level
						//TODO igonore (relative) small floors
						//TODO evaluate if floor is flat

						//TODO order floors

					}
				}
			}
		}

	}




	// write to file
	/*
	std::ofstream storageFile;
	storageFile.open(exportPath);
	std::cout << "exporting" << std::endl;
	storageFile << sourceFile;
	std::cout << "exported succesfully" << std::endl;
	storageFile.close();

	std::cout << "last line executed" << std::endl;
	*/
	return 0;

}